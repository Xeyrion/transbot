import sys
from pathlib import Path
import unittest
import os
import subprocess
import tempfile
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'recording'))
from stream_health import StreamHealth,ODOM_TF
import clock_check

class ClockHealthTests(unittest.TestCase):
    def feed(self,h,now,overrides=None):
        for topic in h.required:
            h.observe(topic,now,(overrides or {}).get(topic,None if topic=='/tf' else 100.+now))

    @unittest.skipUnless(os.name=='posix','POSIX flock integration')
    def test_preflight_background_process_does_not_inherit_recording_lock(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            source=(ROOT/'recording/record_full.sh').read_text(encoding='utf-8')
            source=source.replace('/tmp/transbot_record.',d+'/record.').replace('source /opt/ros/melodic/setup.bash',':')
            source=source.replace('if [ -f "$HOME/','if [ -f "/nonexistent/')
            (p/'record_full.sh').write_text(source)
            for helper in ['session_status.py','stop_recording.py']:
                (p/helper).write_bytes((ROOT/'recording'/helper).read_bytes())
            (p/'preflight.sh').write_text('#!/bin/bash\nsleep 30 >/dev/null 2>&1 &\necho $! > '+d+'/child\nexit 0\n')
            recorder=p/'auto_record_cam.sh';recorder.write_text('#!/bin/bash\nexit 0\n');recorder.chmod(0o755)
            try:
                subprocess.check_call(['bash',str(p/'record_full.sh'),'--launch-only'],stdout=subprocess.DEVNULL)
                result=subprocess.run(['flock','-n',str(p/'record.lock'),'true'])
                self.assertEqual(result.returncode,0)
            finally:
                if (p/'child').exists():
                    try:os.kill(int((p/'child').read_text()),15)
                    except ProcessLookupError:pass

    def test_normal_and_tf_is_required(self):
        h=StreamHealth('nav',0,grace=0)
        for i in range(30):self.feed(h,i*.05)
        self.assertEqual(h.snapshot(1.45)['state'],'healthy')
        self.assertIn(ODOM_TF,h.required)
        self.assertNotIn(ODOM_TF,h.subscriptions)

    def test_tf_clock_regression(self):
        h=StreamHealth('nav',0);self.feed(h,0)
        self.feed(h,.05,{ODOM_TF:99.4})
        self.assertIn('header_regression:'+ODOM_TF,h.snapshot(.05)['faults'])

    def test_small_forward_jump(self):
        h=StreamHealth('nav',0);self.feed(h,0)
        self.feed(h,.05,{'/odom':100.696171})
        self.assertIn('header_clock_jump:/odom',h.snapshot(.05)['faults'])

    def test_frozen_header_with_live_messages(self):
        h=StreamHealth('nav',0)
        for i in range(21):self.feed(h,i*.05,{'/odom':100.})
        self.assertIn('frozen_header:/odom',h.snapshot(1.)['faults'])

    def test_old_rtc_is_rejected_without_changing_clock(self):
        with patch.object(clock_check.time,'time',return_value=1629987800.), \
             patch.object(clock_check.time,'monotonic',side_effect=[0.,1.]), \
             patch.object(clock_check.xmlrpc.client,'ServerProxy',side_effect=OSError()), \
             patch.object(clock_check.subprocess,'check_output',side_effect=OSError()):
            self.assertEqual(clock_check.check(.1)['state'],'failed')

    def test_simulated_clock_is_rejected(self):
        with patch.object(clock_check.time,'monotonic',side_effect=[0.,1.]), \
             patch.object(clock_check.xmlrpc.client,'ServerProxy') as master, \
             patch.object(clock_check.subprocess,'check_output',return_value=b'Leap status: Normal'):
            master.return_value.getParam.return_value=(1,'',True)
            self.assertIn('use_sim_time_true_on_live_recorder',clock_check.check(.1)['faults'])

if __name__=='__main__':unittest.main()
