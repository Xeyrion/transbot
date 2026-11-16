import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from mapping_job import recording_groups,new_job,map_artifacts,publish,source_snapshot,MappingJob
from viewer_core import association,mapped_window


def result_fixture(job):
    from PIL import Image
    result=job/'result';result.mkdir()
    image=Image.new('L',(2,2),205);image.putpixel((0,0),0);image.putpixel((1,1),254)
    image.save(str(result/'map.pgm'))
    (result/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    (result/'trajectory.bag').write_bytes(b'trajectory')
    (result/'association.json').write_text(json.dumps(dict(yaml='map.yaml',trajectory='trajectory.bag',
        replace_map_odom=True,record_bounds=[10.,20.])))


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.first=self.root/'cam_20261115_120000_0.bag';self.first.write_bytes(b'original')
        self.second=self.root/'cam_20261115_120000_1.bag';self.second.write_bytes(b'original2')
        self.files=[str(self.first),str(self.second)];self.pointer=Path(str(self.first)+'.map.json')

    def tearDown(self):self.tmp.cleanup()

    def test_groups_exclude_derived_and_keep_numeric_order(self):
        (self.root/'cam_20261115_120000_10.bag').write_bytes(b'x')
        (self.root/'cam_20261115_120000_2.bag').write_bytes(b'x')
        (self.root/'cam_20261115_120000_0.bag.mapping.bag').write_bytes(b'x')
        (self.root/'cam_20261115_130000_0.bag').write_bytes(b'x')
        groups=recording_groups(str(self.root))
        self.assertEqual(len(groups),2)
        self.assertEqual([Path(p).name.rsplit('_',1)[1] for p in groups[0]],['0.bag','1.bag','2.bag','10.bag'])

    def test_active_recording_is_refused_before_creating_job(self):
        (self.root/'test.bag.active').touch()
        with self.assertRaises(ValueError):new_job(str(self.first))
        self.assertFalse((self.root/'.bagtools_maps').exists())

    def test_success_switches_map_and_retains_previous_pointer(self):
        previous=b'{"old":true}';self.pointer.write_bytes(previous)
        job=new_job(str(self.first));result_fixture(job)
        publish(str(self.first),job,source_snapshot(self.files),previous)
        self.assertEqual((job/'previous_association.json').read_bytes(),previous)
        linked=association(str(self.second))
        self.assertTrue(linked['replace_map_odom'])
        self.assertEqual(linked['record_bounds'],[10.,20.])
        self.assertTrue(Path(linked['trajectory']).is_file())
        self.assertEqual(self.first.read_bytes(),b'original')

    def test_failed_outputs_never_replace_active_map(self):
        old=b'old map pointer';self.pointer.write_bytes(old)
        job=new_job(str(self.first));result_fixture(job);(job/'result/trajectory.bag').unlink()
        with self.assertRaises(ValueError):publish(str(self.first),job,source_snapshot(self.files),old)
        self.assertEqual(self.pointer.read_bytes(),old)

    def test_source_growth_and_external_pointer_change_are_refused(self):
        job=new_job(str(self.first));result_fixture(job);sources=source_snapshot(self.files)
        self.second.write_bytes(b'changed')
        with self.assertRaises(ValueError):publish(str(self.first),job,sources,None)
        self.assertFalse(self.pointer.exists())
        sources=source_snapshot(self.files);self.pointer.write_bytes(b'external new pointer')
        with self.assertRaises(ValueError):publish(str(self.first),job,sources,None)
        self.assertEqual(self.pointer.read_bytes(),b'external new pointer')

    def test_cancel_before_commit_preserves_map(self):
        self.pointer.write_bytes(b'old');job=new_job(str(self.first));result_fixture(job);(job/'cancel').touch()
        with self.assertRaises(KeyboardInterrupt):publish(str(self.first),job,source_snapshot(self.files),b'old')
        self.assertEqual(self.pointer.read_bytes(),b'old')

    @unittest.skipIf(os.name=='nt','ROS mapping worker runs on Linux')
    def test_existing_map_requires_confirmation_in_worker(self):
        (self.root/'map_saved.yaml').write_text('image: old.pgm')
        self.assertTrue(map_artifacts(str(self.first)))
        job=new_job(str(self.first))
        with self.assertRaises(ValueError):MappingJob(str(self.first),str(job),False).run()
        self.assertFalse((job/'result').exists())

    def test_map_bounds_clip_or_degrade_without_borrowing_old_map(self):
        linked=dict(replace_map_odom=True,record_bounds=[15.,35.])
        self.assertEqual(mapped_window(10.,20.,linked),(15.,20.,True,True))
        self.assertEqual(mapped_window(20.,30.,linked),(20.,30.,True,False))
        self.assertEqual(mapped_window(40.,50.,linked),(40.,50.,False,False))


if __name__=='__main__':unittest.main()
