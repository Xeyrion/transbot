import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch,MagicMock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'semantic')); sys.path.insert(0,str(ROOT/'recording')); sys.path.insert(0,str(ROOT))
from stream_health import StreamHealth
from viewer_core import session_files,association


class HealthTests(unittest.TestCase):
    @unittest.skipUnless(os.name=='posix','POSIX signal integration')
    def test_cancel_interrupts_preflight_without_starting_recorder(self):
        import time
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); source=(ROOT/'recording/record_full.sh').read_text(encoding='utf-8')
            source=source.replace('/tmp/transbot_record.',d+'/lock.').replace('source /opt/ros/melodic/setup.bash',':')
            source=source.replace('if [ -f "$HOME/','if [ -f "/nonexistent/')
            (p/'record_full.sh').write_text(source)
            for helper in ['session_status.py','stop_recording.py']:
                (p/helper).write_bytes((ROOT/'recording'/helper).read_bytes())
            (p/'preflight.sh').write_text('#!/bin/bash\necho ready > '+d+'/ready\ntrap "echo stopped > '+d+'/stopped; exit 143" TERM\nwhile :; do sleep .05; done\n')
            auto=p/'auto_record_cam.sh'; auto.write_text('#!/bin/bash\ntouch '+d+'/started\n'); auto.chmod(0o755)
            child=subprocess.Popen(['bash',str(p/'record_full.sh'),'--no-launch','-o',str(p/'out')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            try:
                deadline=time.monotonic()+5
                while not (p/'ready').exists() and child.poll() is None and time.monotonic()<deadline: time.sleep(.05)
                self.assertTrue((p/'ready').exists())
                child.terminate(); child.wait(timeout=5)
                self.assertTrue((p/'stopped').exists()); self.assertFalse((p/'started').exists())
            finally:
                if child.poll() is None: child.terminate(); child.wait(timeout=5)
    def test_stop_owns_direct_auto_wrapper_without_duplicate_signals(self):
        import stop_recording
        processes={10:['bash','/tmp/record_full.sh'],11:['bash','/tmp/auto_record_cam.sh'],12:['/opt/ros/lib/rosbag/record','-O','/tmp/bag']}
        with patch.object(stop_recording,'processes',return_value=processes):
            wrappers,recorders=stop_recording.groups()
        self.assertEqual(set(wrappers),{10,11}); self.assertEqual(recorders,[12])
        parents={12:11,11:10,10:1}
        self.assertTrue(stop_recording.managed(12,set(wrappers),lambda p:parents.get(p,1)))
        self.assertFalse(stop_recording.managed(10,set(wrappers),lambda p:parents.get(p,1)))
    def test_health_snapshot_is_immutable_and_faults_latch(self):
        h=StreamHealth('nav',0,grace=0,timeout=1)
        h.observe('/scan',0,10); snap=h.snapshot(0)
        h.observe('/scan',.1,10.1)
        self.assertEqual(snap['streams']['/scan']['count'],1)
        h.observe('/odom',.1,10.1); h.observe('/tf',.1,None)
        self.assertEqual(h.snapshot(.1)['state'],'failed')
    def test_latched_info_fresh_live_and_latched_failure(self):
        h=StreamHealth('full',0,grace=2,timeout=1)
        for t in h.required: h.observe(t,0,None if t.endswith('camera_info') else 10)
        self.assertEqual(h.snapshot(.5)['state'],'healthy')
        for t in ['/camera/rgb/image_raw','/scan','/odom','/tf']: h.observe(t,2.1,12)
        self.assertIn('stale:/camera/depth/image_raw',h.snapshot(2.2)['faults'])
        h.observe('/camera/depth/image_raw',2.3,9)
        self.assertTrue(any('header_regression' in s for s in h.snapshot(2.3)['faults']))

    def test_missing_stream_after_grace(self):
        h=StreamHealth('nav',0,grace=2)
        self.assertEqual(h.snapshot(1)['state'],'starting')
        self.assertEqual(h.snapshot(3)['state'],'failed')


class SessionTests(unittest.TestCase):
    def test_bundle_rejects_partial_versions_and_quotes_are_plain_arguments(self):
        import recorder_bundle,remote_start,shutil
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name in recorder_bundle.NAMES:
                source=ROOT/name if name=='calibration_schema.py' else ROOT/'recording'/name
                shutil.copy2(str(source),str(root/name))
            payload=dict(protocol_version=1,files=recorder_bundle.bundle(),root=str(root),duration=1,
                         out='/tmp/space quote\" $(not_executed)',name='demo')
            with patch.object(remote_start,'active_recorders',return_value=[]),patch.object(remote_start.subprocess,'Popen') as popen,patch('builtins.open',MagicMock()):
                popen.return_value.pid=123
                self.assertTrue(remote_start.start(payload)['bundle_verified'])
                self.assertEqual(popen.call_args[0][0][5],payload['out'])
                self.assertNotIn('shell',popen.call_args[1])
            (root/'auto_record_cam.sh').write_text('old version')
            with self.assertRaises(ValueError): remote_start.start(payload)
    def test_candidate_time_windows_exclude_clock_regression(self):
        from time_segments import review
        streams={t:[(float(i),float(i if i<10 else i-2)) for i in range(25)] for t in ['rgb','depth','scan','odom']}
        result=review(streams,max_gap=2,margin=.5)
        self.assertTrue(result['segments']); self.assertTrue(any(i['kind']=='clock_regression' for i in result['issues']))
        self.assertTrue(all(not s['record_start']<10<s['record_end'] for s in result['segments']))

    def test_calibration_schema_does_not_assume_verified(self):
        from calibration_schema import load_calibration
        with tempfile.TemporaryDirectory() as d:
            p=Path(d,'c.json'); data=dict(id='x',transforms=[dict(parent='base_link',child='camera_link',xyz=[0,0,0],quaternion_xyzw=[0,0,0,1])])
            p.write_text(json.dumps(data)); self.assertTrue(load_calibration(str(p))['estimated'])
            data['transforms'][0]['quaternion_xyzw']=[0,0,0,2]; p.write_text(json.dumps(data))
            with self.assertRaises(ValueError): load_calibration(str(p))
    def test_numeric_order_and_ambiguous_directory(self):
        with tempfile.TemporaryDirectory() as d:
            for n in [0,2,10]: Path(d,'cam_20261115_120000_%d.bag'%n).write_bytes(b'fixture')
            self.assertEqual([Path(p).name for p in session_files(d)],['cam_20261115_120000_%d.bag'%n for n in [0,2,10]])
            Path(d,'cam_20261115_130000_0.bag').write_bytes(b'fixture')
            with self.assertRaises(ValueError): session_files(d)

    def test_conflicting_static_dynamic_and_cycles_are_rejected(self):
        import pose
        t=pose.Tf(); identity=pose.transform([0,0,0],[0,0,0,1])
        t.add('a','b',0,identity,static=True); t.add('a','b',1,pose.transform([1,0,0],[0,0,0,1]))
        with self.assertRaises(ValueError): t.finish()
        t=pose.Tf(); t.add('a','b',0,identity,static=True); t.add('b','a',0,identity,static=True)
        with self.assertRaises(ValueError): t.finish()

    def test_association_rejects_subset_of_session(self):
        with tempfile.TemporaryDirectory() as d:
            first=Path(d,'cam_20261115_120000_0.bag'); first.write_bytes(b'x')
            Path(d,'cam_20261115_120000_1.bag').write_bytes(b'x')
            manifest=Path(d,'association.json'); st=first.stat()
            manifest.write_text(json.dumps(dict(sources=[dict(name=first.name,size=st.st_size,mtime=st.st_mtime)],yaml='x.yaml',trajectory=None)))
            with self.assertRaises(ValueError): association(str(first),str(manifest))


class NumericalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import numpy as np
            import camera_geometry as cg
            # recording/ retains a historical helper with the same module name.
            import importlib.util
            spec=importlib.util.spec_from_file_location('semantic_detect_project_under_test',
                                                       str(ROOT/'semantic/detect_project.py'))
            dp=importlib.util.module_from_spec(spec); spec.loader.exec_module(dp)
            import cluster_objects as co
            cls.np,cls.cg,cls.dp,cls.co=np,cg,dp,co
        except ImportError as e: raise unittest.SkipTest(str(e))

    def test_lens_inverse_and_ambiguous_depth_rejection(self):
        np=self.np
        x=np.array([-.3,0,.3]); y=np.array([-.2,.1,.2]); d=[.1,-.02,.001,-.002,.005]
        xx,yy=self.cg.distort(x,y,d); rx,ry=self.cg.undistort(xx,yy,d)
        self.assertTrue(np.allclose(x,rx,atol=1e-7)); self.assertTrue(np.allclose(y,ry,atol=1e-7))
        depth=np.ones((20,20)); depth[:,10:]=3
        k=[100,0,10,0,100,10,0,0,1]; m=np.eye(4).tolist()
        self.assertIsNone(self.dp.project(depth,k,m,0,0,20,20))

    def test_height_separates_instances(self):
        groups=self.co.cluster([dict(x=0,y=0,z=z) for z in [0,1]],.6)
        self.assertEqual(len(groups),2)

    def test_rotated_raster_uses_world_origin_and_yaw(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); Image.fromarray(self.np.array([[0,254],[205,0]],dtype='uint8')).save(str(p/'map.pgm'))
            (p/'map.yaml').write_text('image: map.pgm\nresolution: 1\norigin: [2,0,1.5707963267948966]\n')
            canvas=Image.new('RGB',(4,4),(13,17,23))
            self.co.draw_map(canvas,str(p/'map.yaml'),lambda x,y:(x,4-y),1)
            self.assertEqual(canvas.getpixel((0,3)),(229,231,235))
            self.assertEqual(canvas.getpixel((0,2)),(31,41,55))
            self.assertEqual(canvas.getpixel((1,3)),(13,17,23))

    def test_session_split_validation_and_no_overwrite(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); src=root/'src'
            for group in ['a','b','c']:
                (src/group).mkdir(parents=True)
                for i in range(2):
                    Image.new('RGB',(10,10),(ord(group),i,0)).save(str(src/group/('%s.png'%i)))
                    (src/group/('%s.txt'%i)).write_text('0 .5 .5 .2 .2\n')
            cmd=[sys.executable,str(ROOT/'semantic/make_yolo_dataset.py'),str(src),str(root/'out'),'--names','chair']
            p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(p.returncode,0,p.stderr.decode())
            data=json.loads((root/'out/split.json').read_text()); memberships={}
            for r in data['records']: memberships.setdefault(r['group'],set()).add(r['split'])
            self.assertTrue(all(len(s)==1 for s in memberships.values()))
            before=(root/'out/split.json').read_bytes()
            p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertNotEqual(p.returncode,0); self.assertEqual((root/'out/split.json').read_bytes(),before)


if __name__=='__main__': unittest.main()
