import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'semantic'))
sys.path.insert(0, str(ROOT/'recording'))
import pose
from sampling import HeaderIndex, ClockError
from session_finalize import finalize, REQUIRED, EXPECTED_TYPES
import record_guard


class TimeGeometryTests(unittest.TestCase):
    def test_header_pairing_ignores_record_latency_and_rejects_missing(self):
        index = HeaderIndex()
        index.add(10., 10.5, 'a', 1); index.add(10.03, 10.6, 'a', 2)
        self.assertEqual(index.nearest(10.029, .01)['seq'], 2)
        self.assertIsNone(index.nearest(11., .033))
        with self.assertRaises(ClockError): index.add(9., 12., 'a')

    def test_se3_midpoint_and_age(self):
        tf = pose.Tf()
        tf.add('world', 'camera', 1., pose.transform([0,0,0],pose.yaw_quat(0)))
        tf.add('world', 'camera', 1.4, pose.transform([2,0,0],pose.yaw_quat(math.pi/2)))
        tf.finish()
        p = pose.apply(tf.chain_to('camera',1.2,'world'),[1,0,0])
        self.assertAlmostEqual(p[0],1+math.sqrt(.5)); self.assertAlmostEqual(p[1],math.sqrt(.5))
        self.assertIsNone(tf.chain_to('camera',1000.,'world'))

    def test_static_age_conflicts_and_explicit_estimates(self):
        tf=pose.Tf(); tf.add('world','camera',0.,pose.transform([1,0,0],pose.yaw_quat(0)),static=True)
        tf.finish(); self.assertEqual(tf.chain_to('camera',10000.,'world')[0][3],1)
        tf.add('other','camera',0.,pose.transform([0,0,0],pose.yaw_quat(0)),static=True)
        with self.assertRaises(ValueError): tf.finish()
        empty=pose.Tf(); self.assertEqual(empty.fill_known(),[]); self.assertFalse(empty.edges)
        self.assertEqual(len(empty.fill_known(allow=True)),2)

    def test_rotation_shortest_arc(self):
        a=pose.transform([0,0,0],pose.yaw_quat(math.radians(179)))
        b=pose.transform([0,0,0],pose.yaw_quat(math.radians(-179)))
        m=pose.interpolate(a,b,.5)
        self.assertAlmostEqual(m[0][0],-1.,places=6)


class RecordingOutcomeTests(unittest.TestCase):
    def outcome(self, rc=0, active=False, counts=None, stopped=False, timeout=False):
        with tempfile.TemporaryDirectory() as d:
            Path(d,'cam_test_0.bag').write_bytes(b'fixture')
            Path(d,'cam_test.health.json').write_text(json.dumps(dict(state='healthy',faults=[])))
            if active: Path(d,'cam_test_1.bag.active').write_bytes(b'fixture')
            return finalize(d,'test','full',rc,stopped,timeout,
                            'requested' if stopped else 'natural',
                            probe=lambda files: dict(topic_counts=counts if counts is not None else dict.fromkeys(REQUIRED['full'],10),
                                topic_types={k:[EXPECTED_TYPES[k]] for k in REQUIRED['full']}))

    def test_good_abnormal_and_expected_interrupt(self):
        self.assertTrue(self.outcome()['complete'])
        self.assertFalse(self.outcome(rc=7)['complete'])
        self.assertTrue(self.outcome(rc=130,stopped=True)['complete'])
        self.assertFalse(self.outcome(rc=130)['complete'])

    def test_missing_sensor_active_tail_and_timeout(self):
        r=self.outcome(counts={'/scan':1})
        self.assertTrue(r['container_finalized']); self.assertFalse(r['complete'])
        self.assertIn('/odom',r['data_quality']['missing'])
        self.assertFalse(self.outcome(active=True)['container_finalized'])
        self.assertFalse(self.outcome(timeout=True)['complete'])

    def test_failed_preflight_cannot_be_qualified(self):
        with patch.dict(os.environ,{'BAGTOOLS_PREFLIGHT_OK':'0'}):
            self.assertFalse(self.outcome()['complete'])

    def test_guard_clock_jump_and_disk_reserve(self):
        with patch.object(record_guard,'running',return_value=True), \
             patch.object(record_guard.time,'monotonic',side_effect=[0.,1.]), \
             patch.object(record_guard.time,'time',side_effect=[100.,102.]):
            self.assertEqual(record_guard.monitor(1,'/tmp/test',10,0),'clock_jump')
        with patch.object(record_guard,'running',return_value=True), \
             patch.object(record_guard.time,'monotonic',side_effect=[0.,1.]), \
             patch.object(record_guard.time,'time',side_effect=[100.,101.]), \
             patch.object(record_guard.shutil,'disk_usage') as usage:
            usage.return_value.free=0
            self.assertEqual(record_guard.monitor(1,'/tmp/test',10,1),'disk_floor')


class FusionAndMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import cluster_objects
            cls.co=cluster_objects
        except ImportError as exc:
            raise unittest.SkipTest(str(exc))

    def test_transitive_bridge_is_not_one_instance(self):
        groups=self.co.cluster([dict(x=x,y=0) for x in [0,.5,1.,1.5]],.6)
        self.assertEqual([len(g) for g in groups],[2,2])

    def test_same_frame_is_one_vote(self):
        with tempfile.TemporaryDirectory() as d:
            obs=[dict(label='chair',frame=0,x=0,y=0,z=1,score=.9) for _ in range(3)]
            Path(d,'observations.json').write_text(json.dumps(dict(frames=1,observations=obs)))
            result=subprocess.run([sys.executable,str(ROOT/'semantic/cluster_objects.py'),d],
                stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(result.returncode,0,result.stderr.decode())
            self.assertEqual(json.loads(Path(d,'objects.json').read_text())['objects'],[])

    def test_transferred_absolute_map_path_falls_back_beside_yaml(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as d:
            Image.new('L',(2,2),254).save(str(Path(d,'map.pgm')))
            Path(d,'map.yaml').write_text('image: /home/jetson/old/map.pgm\nresolution: 1\norigin: [0,0,0]\n')
            canvas=Image.new('RGB',(10,10))
            self.assertTrue(self.co.draw_map(canvas,str(Path(d,'map.yaml')),lambda x,y:(x,10-y),2))


if __name__ == '__main__': unittest.main()
