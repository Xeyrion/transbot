import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'recording'))
import robot_prepare as prepare
import record_guard


class PreparationTests(unittest.TestCase):
    def test_reject_invalid_control_time(self):
        for value in [True,float('nan'),float('inf'),946684800.,'2026']:
            with self.assertRaises(ValueError):prepare.valid_utc(value)
        self.assertEqual(prepare.valid_utc(1794869000.),1794869000.)

    def test_clock_requests_fresh_time_after_stopping_ros(self):
        service=prepare.Preparation.__new__(prepare.Preparation);service.root=Path('/fixture')
        calls=[]
        service.ask_time=Mock(side_effect=[1794869005.,1794869020.])
        service.stop_sources=lambda:calls.append('stop')
        def run(args,**kwargs):calls.append(args);return Mock(returncode=0)
        service.run=run
        service.launch_sources=lambda:calls.append('start')
        with patch('robot_prepare.time.time',return_value=1794869000.),patch('robot_prepare.emit'):
            self.assertTrue(service.synchronize())
        self.assertEqual(service.ask_time.call_count,2)
        date=[c for c in calls if isinstance(c,list) and 'date' in c][0]
        self.assertEqual(date[-1],'@1794869020.000000')
        self.assertEqual(calls[0],'stop');self.assertEqual(calls[-1],'start')

    def test_aligned_clock_does_not_restart_sources(self):
        service=prepare.Preparation.__new__(prepare.Preparation);service.root=Path('/fixture')
        service.ask_time=lambda:1794869000.05;service.stop_sources=Mock();service.run=Mock()
        with patch('robot_prepare.time.time',return_value=1794869000.),patch('robot_prepare.emit'):
            self.assertFalse(service.synchronize())
        service.stop_sources.assert_not_called()

    @unittest.skipUnless(os.name=='posix','POSIX shared lock')
    def test_recording_prevents_all_maintenance_before_loading_ros(self):
        with tempfile.TemporaryDirectory() as folder,patch('robot_prepare.read_json',return_value={'action':'clock'}), \
             patch('robot_prepare.verify_bundle',return_value=ROOT/'recording'), \
             patch('robot_prepare.PREPARATION_LOCK',str(Path(folder)/'lock')), \
             patch('robot_prepare.active_recorders',return_value=[123]),patch('robot_prepare.Preparation') as service:
            with self.assertRaisesRegex(RuntimeError,'Recording/finalization'):prepare.serve()
            service.assert_not_called()

    @unittest.skipUnless(os.name=='posix','POSIX shared lock')
    def test_maintenance_lock_prevents_second_preparation(self):
        import fcntl
        with tempfile.TemporaryDirectory() as folder:
            path=str(Path(folder)/'lock')
            with open(path,'a') as handle:
                fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with patch('robot_prepare.read_json',return_value={'action':'all'}), \
                     patch('robot_prepare.verify_bundle',return_value=ROOT/'recording'), \
                     patch('robot_prepare.PREPARATION_LOCK',path),patch('robot_prepare.Preparation') as service:
                    with self.assertRaisesRegex(RuntimeError,'Recording or preparation'):prepare.serve()
                    service.assert_not_called()

    def test_guard_preserves_trigger_in_separate_runtime_path(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);health=root/'runtime-health.json';evidence=root/'runtime-evidence.json'
            health.write_text(json.dumps(dict(state='healthy',updated_monotonic=7.,observer_pid=999999)))
            with patch.object(record_guard,'running',return_value=True), \
                 patch.object(record_guard.time,'monotonic',side_effect=[0.,11.]), \
                 patch.object(record_guard.time,'time',side_effect=[100.,111.,111.]):
                reason=record_guard.monitor(1,str(root/'bag'),60,0,health_path=str(health),evidence_path=str(evidence))
            self.assertEqual(reason,'health_observer_failed')
            self.assertEqual(json.loads(evidence.read_text())['heartbeat_age_s'],4.)
            self.assertFalse((root/'bag.guard.json').exists())
