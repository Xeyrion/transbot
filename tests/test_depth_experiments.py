import importlib.util
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'semantic'))


class DepthExperiments(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import numpy as np
            import cv2
            from depth_projection import project_depth
            from cluster_objects import view_support,cluster
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))
        cls.np=np;cls.project=staticmethod(project_depth);cls.support=staticmethod(view_support);cls.cluster=staticmethod(cluster)

    def setUp(self):
        self.k=[100.,0,20.,0,100.,20.,0,0,1.]
        self.m=self.np.eye(4).tolist()

    def test_legacy_planar_projection_and_rejection_reasons(self):
        depth=self.np.full((40,40),2.)
        result=self.project(depth,self.k,self.m,[0,0,40,40])
        self.assertIsNone(result['reason'])
        self.assertAlmostEqual(result['point'][0],-.01)
        self.assertEqual(result['point'][2:], [2.,2.])
        depth[:,20:]=4.
        self.assertEqual(self.project(depth,self.k,self.m,[0,0,40,40])['reason'],'mixed_depth')
        depth[:]=0
        self.assertEqual(self.project(depth,self.k,self.m,[0,0,40,40])['reason'],'insufficient_valid_depth')

    def test_mask_excludes_dominant_background_and_uses_supported_pixel(self):
        depth=self.np.full((40,40),5.)
        mask=self.np.zeros((40,40),bool);mask[5:15,5:15]=True;depth[mask]=2.
        result=self.project(depth,self.k,self.m,[0,0,40,40],mode='mask-surface',instance_mask=mask)
        self.assertIsNone(result['reason'])
        self.assertEqual(result['point'][2],2.)
        u,v=result['quality']['pixel_uv'];self.assertTrue(mask[int(v),int(u)])
        self.assertEqual(result['quality']['valid_pixels'],100)

    def test_mask_holes_are_not_filled_or_chosen_as_measurements(self):
        depth=self.np.full((40,40),2.)
        mask=self.np.ones((40,40),bool);mask[12:28,12:28]=False
        depth[~mask]=0
        result=self.project(depth,self.k,self.m,[0,0,40,40],mode='mask-surface',instance_mask=mask)
        u,v=result['quality']['pixel_uv']
        self.assertTrue(mask[int(v),int(u)])
        self.assertEqual(result['point'][2],2.)
        self.assertTrue(self.np.all(depth[~mask]==0))

    def test_equally_supported_separated_surfaces_remain_ambiguous(self):
        depth=self.np.full((40,40),2.);depth[:,20:]=4.
        result=self.project(depth,self.k,self.m,[0,0,40,40],mode='surface')
        self.assertIsNone(result['point'])
        self.assertEqual(result['reason'],'ambiguous_surfaces')

    def test_mask_requirement_dimensions_and_invalid_rays(self):
        depth=self.np.full((40,40),2.)
        self.assertEqual(self.project(depth,self.k,self.m,[0,0,40,40],mode='mask-surface')['reason'],'missing_instance_mask')
        with self.assertRaisesRegex(ValueError,'RGB depth grid'):
            self.project(depth,self.k,self.m,[0,0,40,40],mode='mask-surface',instance_mask=self.np.ones((20,20)))
        self.assertEqual(self.project(depth,self.k,self.m,[-50,0,10,40])['reason'],'invalid_roi')

    def test_repeated_frames_do_not_manufacture_viewpoint_support(self):
        points=[dict(t=i/3.,camera_xyz=[0,0,0],view_direction=[0,0,1]) for i in range(10)]
        ok,quality=self.support(points,1.,.15,8.)
        self.assertFalse(ok);self.assertEqual(quality['baseline_m'],0.)
        points[-1]['camera_xyz']=[.2,0,0]
        self.assertTrue(self.support(points,1.,.15,8.)[0])
        points[-1]['camera_xyz']=[0,0,0]
        points[-1]['view_direction']=[.2,0,.98]
        self.assertTrue(self.support(points,1.,.15,8.)[0])
        points[-1]['t']=.1
        self.assertFalse(self.support(points,10.,.15,8.)[0])

    def test_cached_predictions_validate_pixels_time_and_confidence_coverage(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);frame=root/'0000';frame.mkdir()
            Image.new('RGB',(40,40),(10,20,30)).save(str(frame/'rgb.png'))
            self.np.save(str(frame/'depth.npy'),self.np.full((40,40),2.))
            meta=dict(k=self.k,t_map_cam=self.m,world_frame='odom',geometry={},
                      rgb_record=100.,stamp=99.99,depth_stamp=99.98)
            (frame/'meta.json').write_text(json.dumps(meta))
            (root/'sampling.json').write_text(json.dumps(dict(state='complete')))
            record=dict(rgb_sha256=hashlib.sha256((frame/'rgb.png').read_bytes()).hexdigest(),
                        rgb_record=100.,detections=[dict(label='chair',score=.9,bbox_xywh=[0,0,40,40])])
            predictions=dict(confidence_floor=.35,frames={'0000':record})
            path=root/'cache.json'
            def invoke(conf='.35'):
                path.write_text(json.dumps(predictions))
                return subprocess.run([sys.executable,str(ROOT/'semantic/detect_project.py'),str(root),
                    '--predictions',str(path),'--conf',conf],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
            result=invoke();self.assertEqual(result.returncode,0,result.stderr)
            obs=json.loads((root/'observations.json').read_text());self.assertEqual(len(obs['observations']),1)
            self.assertEqual(obs['observations'][0]['dist'],2.)
            self.assertIn('lower confidence',invoke('.25').stderr)
            record['rgb_record']=101.
            self.assertIn('different RGB frame',invoke().stderr)
            record['rgb_record']=100.;record['rgb_sha256']='0'*64
            self.assertIn('different RGB frame',invoke().stderr)

    def test_covisible_chairs_cannot_merge_even_at_close_surface_points(self):
        a=dict(label='chair',x=0,y=0,z=.5,frame=0,observation_id='a',bbox_xywh=[0,0,50,80])
        b=dict(a,x=.1,observation_id='b',bbox_xywh=[60,0,50,80])
        self.assertEqual(len(self.cluster([a,b],.45,'point')),1)
        self.assertEqual(len(self.cluster([a,b],.45,'instance')),2)
        # A seat and back sampled in different frames can support one chair.
        b.update(frame=1,x=.05,z=1.1)
        self.assertEqual(len(self.cluster([a,b],.45,'point')),2)
        self.assertEqual(len(self.cluster([a,b],.45,'instance')),1)


if __name__=='__main__':unittest.main()
