import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'recording'))
import record_probe as probe


class ProbeTests(unittest.TestCase):
    def test_idle_sensor_rates_do_not_require_recording_health(self):
        snapshot=dict(updated_monotonic=10.,streams={t:dict(count=100,last_monotonic=10.) for t in probe.TOPICS},
                      rates={probe.TOPICS[0]:30.,probe.TOPICS[1]:29.,probe.TOPICS[2]:7.5})
        self.assertEqual(probe.topic_rates(snapshot,10.1)[probe.TOPICS[0]],30.)
        self.assertEqual(probe.topic_rates(snapshot,14.)[probe.TOPICS[0]],-1.)
        snapshot['updated_monotonic']=14.
        self.assertEqual(probe.topic_rates(snapshot,14.)[probe.TOPICS[0]],0.)

    def test_no_stream_is_zero_when_observer_is_alive(self):
        snapshot=dict(updated_monotonic=10.,streams={},rates={})
        self.assertEqual(set(probe.topic_rates(snapshot,10.1).values()),{0.})

    def test_recorder_binary_and_frontend_share_one_session(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix=str(Path(folder)/'cam_test.bag')
            procs={1:['python2','/opt/ros/melodic/bin/rosbag','record','-O',prefix],
                   2:['/opt/ros/melodic/lib/rosbag/record','-O',prefix],
                   3:['bash','-c','echo rosbag record -O fake']}
            self.assertEqual(probe.bag_state(procs)[0],prefix)
            self.assertEqual(probe.bag_state(procs)[3],1)

    def test_log_tail_is_bounded_and_missing_bag_is_safe(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'log';p.write_bytes(b'x'*100000+b'final')
            self.assertEqual(probe.read_file(str(p),5),'final')
            self.assertEqual(probe.file_size(str(p/'missing')),0)

    def test_preparation_and_finalization_are_not_idle(self):
        procs={12:['bash','/home/jetson/record_full.sh']}
        self.assertEqual(probe.recording_phase(procs,[],{}),'preparing')
        phase=dict(owner_pid=12,phase='finalizing')
        self.assertEqual(probe.recording_phase(procs,['bag.active'],phase),'finalizing')
        self.assertEqual(probe.recording_phase({},[],phase),'idle')
