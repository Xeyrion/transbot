import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_display import Viewport,raster_view,grid_bounds,export_final_map,gallery_image_path


class MapDisplayTests(unittest.TestCase):
    def test_rectangular_view_has_equal_physical_scale_on_both_axes(self):
        view=Viewport((3.,-2.),12.,900,450)
        self.assertEqual(view.pixel(3,-2),(450.,225.))
        self.assertEqual(view.pixel(4,-1),(525.,150.))
        self.assertIsNone(view.pixel(3,2))
        self.assertEqual(view.pixel(3,2,False),(450.,-75.))
        fitted=Viewport.fit((-5,-2,7,3),900,450)
        self.assertIsNotNone(fitted.pixel(-5,-2))
        self.assertIsNotNone(fitted.pixel(7,3))

    def test_rotated_grid_and_overlay_share_the_same_coordinates(self):
        image=Image.new('RGB',(2,2))
        colors=[(255,0,0),(0,255,0),(0,0,255),(255,255,0)];image.putdata(colors)
        info=dict(origin=[2.,0.,math.pi/2],resolution=1.)
        bounds=grid_bounds(info,image)
        for actual,expected in zip(bounds,[0.,0.,2.,2.]):self.assertAlmostEqual(actual,expected)
        view=Viewport((1,1),2,2,2)
        rendered=raster_view(image,info,view)
        self.assertEqual(list(rendered.getdata()),[colors[1],colors[3],colors[0],colors[2]])
        # The green source cell centre is world (.5, 1.5), upper left.
        self.assertEqual(view.pixel(.5,1.5),(.5,.5))

    def test_standalone_export_uses_all_saved_poses_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            run=dict(state='complete',run_id='test_run',world_frame='odom',geometry={},
                     sources=[dict(path='/bags/session_20261117_071239/cam_20261117_071320_0.bag')])
            objects=dict(world_frame='odom',geometry={},objects=[dict(label='chair',x=2.,y=1.,z=.5)])
            (root/'run.json').write_text(json.dumps(run));(root/'objects.json').write_text(json.dumps(objects))
            for index,x in enumerate([-3.,4.]):
                folder=root/('%04d'%index);folder.mkdir()
                matrix=[[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
                (folder/'meta.json').write_text(json.dumps(dict(world_frame='odom',t_world_optical=matrix)))
            before=(root/'objects.json').read_bytes()
            gallery=root/'gallery'
            path=export_final_map(str(root),gallery=str(gallery))
            with Image.open(path) as image:self.assertEqual(image.size,(2400,1600))
            self.assertEqual((root/'objects.json').read_bytes(),before)
            self.assertEqual(Path(path).name,'final_semantic_map.png')
            self.assertEqual((gallery/'session_20261117_071239__cam_20261117_071320'/'test_run.png').read_bytes(),
                             Path(path).read_bytes())
            objects['world_frame']='map';(root/'objects.json').write_text(json.dumps(objects))
            with self.assertRaisesRegex(ValueError,'coordinate frames differ'):export_final_map(str(root))

    def test_image_fit_can_enlarge_small_camera_images(self):
        from bag_quality_gui import BagViewer
        image=Image.new('RGB',(20,10),(255,0,0))
        box=BagViewer._fit(image,200,200)
        self.assertEqual(box.getpixel((10,60)),(255,0,0))
        self.assertEqual(box.getpixel((10,10)),(17,17,17))

    def test_gallery_keeps_split_parts_together_and_separate_recordings_apart(self):
        def target(source):
            return Path(gallery_image_path(dict(run_id='same_model', sources=[dict(path=source)]), 'unused'))
        first = target('/bags/session_20261117_071239/cam_20261117_071320_0.bag')
        self.assertEqual(first, target(r'D:\bags\session_20261117_071239\cam_20261117_071320_1.bag'))
        self.assertNotEqual(first.parent, target('/bags/session_20261118_080000/cam_20261118_080112_0.bag').parent)
        self.assertNotEqual(target('/archive/entry.bag').parent, target('/archive/lab.bag').parent)
        self.assertNotEqual(first.parent, target('/bags/session_20261117_071239/cam_20261117_075559_0.bag').parent)

    def test_export_refuses_a_modified_map_background(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);yaml_path=str(root/'map.yaml');geometry=dict(map=dict(yaml=yaml_path))
            (root/'map.yaml').write_text('image: map.pgm\nresolution: 1\norigin: [0,0,0]\n')
            run=dict(state='complete',world_frame='map',geometry=geometry,geometry_sha256={yaml_path:'0'*64})
            (root/'run.json').write_text(json.dumps(run))
            (root/'objects.json').write_text(json.dumps(dict(world_frame='map',geometry=geometry,objects=[])))
            with self.assertRaisesRegex(ValueError,'geometry has changed'):export_final_map(str(root))


if __name__=='__main__':unittest.main()
