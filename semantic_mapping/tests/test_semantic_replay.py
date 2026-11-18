import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_replay import SemanticReplay, discover_runs, draw_detections, label_size, COLORS
from viewer_core import semantic_directory


class SemanticReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.bags = [self.root/('cam_20261117_000000_%d.bag' % i) for i in range(2)]
        for bag in self.bags:
            bag.write_bytes(b'fixture')
        self.runs = self.root/'results'
        self.folder = self.runs/'example'
        self.folder.mkdir(parents=True)
        self.run = dict(state='complete', world_frame='odom', geometry={},
                        sources=[dict(path=str(p), size=p.stat().st_size, mtime=p.stat().st_mtime) for p in self.bags],
                        stages=[dict(name='fuse_render', command=['--classes', 'chair,tv'])],
                        model=dict(path='models/yolo26s.pt'))
        self.write('run.json', self.run)
        base = dict(world_frame='odom', geometry={})
        self.write('sampling.json', dict(base, frames=3))
        self.write('observations.json', dict(base, observations=[dict(observation_id='0000:1', dist=2.)], detections=[
            dict(label='chair', source_frame='0000', score=.9, bbox_xywh=[5, 15, 20, 20], projection='accepted', observation_id='0000:1'),
            dict(label='tv', source_frame='0000', score=.7, bbox_xywh=[50, 15, 20, 20], projection='rejected', reason='bad depth'),
            dict(label='person', source_frame='0000', score=.6, bbox_xywh=[80, 15, 20, 20], projection='accepted')]))
        self.write('objects.json', dict(base, objects=[dict(label='chair', x=1., y=2., z=.5, observations=['0000:1'])]))
        for index, record, part in ((0, 10., 0), (1, 12., 0), (2, 13., 1)):
            folder = self.folder/('%04d' % index)
            folder.mkdir()
            (folder/'meta.json').write_text(json.dumps(dict(rgb_record=record, stamp=record-.01,
                depth_stamp=record-.02, world_frame='odom', bag=self.bags[part].name)))
            (folder/'rgb.png').write_bytes(b'PNG')
            (folder/'depth.npy').write_bytes(b'NPY')

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, name, data):
        (self.folder/name).write_text(json.dumps(data))

    def load(self, part=0):
        return SemanticReplay(str(self.folder), str(self.bags[part]))

    def test_exact_samples_bounded_age_and_part_isolation(self):
        replay = self.load()
        self.assertEqual(replay.nearest(10.3)['id'], '0000')
        self.assertEqual(replay.nearest(11.7)['id'], '0001')
        self.assertIsNone(replay.nearest(11.))  # No boxes over an unsampled gap.
        self.assertIsNone(replay.nearest(13.))  # Never borrow from another split.
        self.assertEqual(self.load(1).nearest(13.)['id'], '0002')
        self.assertEqual(replay.nearest(10.)['meta']['stamp'], 9.99)

    def test_detection_status_and_candidate_identity(self):
        replay = self.load()
        payload = replay.payload(replay.nearest(10.))
        self.assertEqual([d['status'] for d in payload['detections']], ['fused', 'rejected', 'excluded'])
        self.assertEqual(payload['detections'][0]['object_number'], 1)
        self.assertEqual(payload['detections'][0]['depth_m'], 2.)
        self.assertEqual(payload['objects'][0]['number'], 1)
        self.assertEqual(replay.payload(None)['detections'], [])
        data = json.loads((self.folder/'objects.json').read_text());data['objects'] = []
        self.write('objects.json', data)
        self.assertEqual(self.load().payload(self.load().nearest(10.))['detections'][0]['status'], 'depth')

    def test_source_fingerprints_and_discovery(self):
        self.assertEqual(len(discover_runs(str(self.bags[0]), str(self.runs))), 1)
        self.bags[1].write_bytes(b'changed recording')
        self.assertEqual(discover_runs(str(self.bags[0]), str(self.runs)), [])
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            self.load()
        self.bags[1].unlink()
        with self.assertRaisesRegex(ValueError, 'different or incomplete'):
            self.load()

    def test_recording_local_discovery_and_legacy_preset(self):
        self.run.update(preset='b1')
        self.write('run.json', self.run)
        target = Path(semantic_directory(str(self.bags[1]))) / self.folder.name
        target.parent.mkdir(parents=True)
        self.folder.rename(target)
        self.assertEqual(discover_runs(str(self.bags[0]))[0]['preset'], 'yolo26s-seg')
        self.assertEqual(discover_runs(str(self.bags[1]))[0]['path'], str(target))
        other = self.root/'different.bag'
        other.write_bytes(b'fixture')
        self.assertEqual(discover_runs(str(other)), [])

    def test_failed_run_geometry_mismatch_missing_sample(self):
        self.run['state'] = 'failed';self.write('run.json', self.run)
        self.assertEqual(discover_runs(str(self.bags[0]), str(self.runs)), [])
        with self.assertRaisesRegex(ValueError, 'not complete'):
            self.load()
        self.run['state'] = 'complete';self.write('run.json', self.run)
        self.write('objects.json', dict(world_frame='map', geometry={}, objects=[]))
        with self.assertRaisesRegex(ValueError, 'different map'):
            self.load()
        self.write('objects.json', dict(world_frame='odom', geometry={}, objects=[]))
        (self.folder/'0000'/'depth.npy').unlink()
        with self.assertRaisesRegex(ValueError, 'Missing semantic sample'):
            self.load()

    def test_geometry_hash_prevents_overlay_on_modified_map(self):
        path = self.root/'trajectory.bag';path.write_bytes(b'changed')
        self.run['geometry_sha256'] = {str(path): '0'*64};self.write('run.json', self.run)
        with self.assertRaisesRegex(ValueError, 'geometry has changed'):
            self.load()

    def test_overlay_is_native_coordinates_and_does_not_mutate_source(self):
        from PIL import Image
        source = Image.new('RGB', (120, 80))
        detections = self.load().payload(self.load().nearest(10.))['detections']
        image = draw_detections(source, detections, labels=False)
        self.assertEqual(source.getpixel((5, 15)), (0, 0, 0))
        self.assertEqual(image.getpixel((5, 15)), COLORS['fused'])
        self.assertEqual(image.getpixel((50, 15)), COLORS['rejected'])
        self.assertEqual(image.getpixel((80, 15)), (0, 0, 0))
        image = draw_detections(source, detections, show_other=True, labels=False)
        self.assertEqual(image.getpixel((80, 15)), COLORS['excluded'])

    def test_bitmap_font_on_old_pillow(self):
        class OldPillowDraw:
            def textbbox(self, xy, text):
                raise ValueError('Only supported for TrueType fonts')

            def textsize(self, text):
                return (30, 10)

        self.assertEqual(label_size(OldPillowDraw(), 'chair'), (34, 12))

    def test_saved_instance_mask_overlay_uses_the_native_grid(self):
        from PIL import Image, ImageDraw
        mask=Image.new('L',(120,80));ImageDraw.Draw(mask).rectangle((10,20,20,25),fill=255)
        path=self.root/'mask.png';mask.save(str(path))
        det=dict(self.load().payload(self.load().nearest(10.))['detections'][0],mask=str(path))
        source=Image.new('RGB',(120,80))
        image=draw_detections(source,[det])
        self.assertNotEqual(image.getpixel((15,23)),(0,0,0))
        self.assertEqual(source.getpixel((15,23)),(0,0,0))
        Image.new('L',(60,40)).save(str(path))
        with self.assertRaisesRegex(ValueError,'dimensions differ'):
            draw_detections(source,[det])


if __name__ == '__main__':
    unittest.main()
