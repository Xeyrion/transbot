import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from viewer_core import TransformIndex, transform_xy, association, world_to_view, session_files,map_manifest_path
sys.path.insert(0, str(ROOT/'recording'))
from session_status import snapshot
from session_catalog import sessions, session_label, session_dir_name, rename_target


class Python2SourceTests(unittest.TestCase):
    """py2 把源码当 ASCII 读，非 ASCII 必须有编码声明，否则 import 就 SyntaxError。

    `viewer_core.py` 是 py2 阅读器（bag_viewer_server）和 py3 界面共用的 —— 往里
    写中文注释而忘了声明，整个阅读器就打不开了。这条守着别再犯。
    """
    PY2_SOURCES = ['viewer_core.py', 'bag_viewer_server.py', 'bag_quality_probe.py',
                   'extract_map.py']

    def test_non_ascii_sources_declare_their_encoding(self):
        for name in self.PY2_SOURCES:
            text = (ROOT/name).read_text(encoding='utf-8')
            if all(ord(ch) < 128 for ch in text):
                continue
            head = text.splitlines()[:2]
            self.assertTrue(any('coding' in line for line in head),
                            '%s 含非 ASCII 但前两行没有编码声明' % name)


class MapAssociationTests(unittest.TestCase):
    def test_loose_legacy_rasters_are_not_used(self):
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)/'cam_20261112_011503_0.bag'
            bag.write_bytes(b'x')
            (Path(d)/'map_20261112_011503.yaml').write_text(
                'image: map_20261112_011503.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
            (Path(d)/'cam_20261112_011503_0.bag.map.json').write_text('{}')
            self.assertIsNone(association(str(bag)))

    def test_rebuilt_map_wins_over_raster(self):
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)/'cam_20261112_011503_0.bag'
            bag.write_bytes(b'x')
            (Path(d)/'map_20261112_011503.yaml').write_text('image: x.pgm\n')
            (Path(d)/'cam_20261112_011503_0.bag.mapping.yaml').write_text('image: x.pgm\n')
            (Path(d)/'cam_20261112_011503_0.bag.mapping.bag').write_bytes(b'tf')
            st = bag.stat()
            current=Path(map_manifest_path(str(bag)));current.parent.mkdir(parents=True)
            current.write_text(json.dumps(dict(
                yaml='cam_20261112_011503_0.bag.mapping.yaml',
                trajectory='cam_20261112_011503_0.bag.mapping.bag',
                sources=[dict(name=bag.name, size=st.st_size, mtime=st.st_mtime)])))
            got = association(str(bag))
            self.assertEqual(got['source'], 'rebuild')
            self.assertTrue(got['trajectory'].endswith('.mapping.bag'))

    def test_association_accepts_a_session_directory(self):
        # 传目录时，来源校验要按解析出的分卷找文件，不能按目录的父级拼
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)/'cam_20261112_011503_0.bag'
            bag.write_bytes(b'x')
            (Path(d)/'cam_20261112_011503_0.bag.mapping.yaml').write_text('image: x.pgm\n')
            (Path(d)/'cam_20261112_011503_0.bag.mapping.bag').write_bytes(b'tf')
            st = bag.stat()
            current=Path(map_manifest_path(str(bag)));current.parent.mkdir(parents=True)
            current.write_text(json.dumps(dict(
                yaml='cam_20261112_011503_0.bag.mapping.yaml',
                trajectory='cam_20261112_011503_0.bag.mapping.bag',
                sources=[dict(name=bag.name, size=st.st_size, mtime=st.st_mtime)])))
            got = association(d)
            self.assertEqual(got['source'], 'rebuild')

    def test_no_map_at_all_is_none(self):
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)/'cam_20261112_011503_0.bag'
            bag.write_bytes(b'x')
            self.assertIsNone(association(str(bag)))


class SessionFileGroupingTests(unittest.TestCase):
    def test_split_bags_group_by_stem(self):
        with tempfile.TemporaryDirectory() as d:
            for n in ('cam_20261111_030000_0.bag', 'cam_20261111_030000_1.bag'):
                (Path(d)/n).write_bytes(b'x')
            got = [os.path.basename(p) for p in session_files(str(Path(d)/'cam_20261111_030000_0.bag'))]
            self.assertEqual(got, ['cam_20261111_030000_0.bag', 'cam_20261111_030000_1.bag'])

    def test_session_directory_is_accepted(self):
        # 传会话目录也能用，且不能把离线建图的 .mapping.bag 当成分卷
        with tempfile.TemporaryDirectory() as d:
            for name in ('cam_20261112_011503_0.bag', 'cam_20261112_011503_1.bag',
                         'cam_20261112_011503_0.bag.mapping.bag'):
                (Path(d)/name).write_bytes(b'x')
            got = [os.path.basename(p) for p in session_files(d)]
            self.assertEqual(got, ['cam_20261112_011503_0.bag',
                                   'cam_20261112_011503_1.bag'])

    def test_single_file_bag_does_not_swallow_the_same_hour(self):
        # 没有 _序号 的是单文件包，末尾那串是时间戳，不是分卷号
        with tempfile.TemporaryDirectory() as d:
            names = ['cam_20261111_014312.bag', 'cam_20261111_020000.bag',
                     'cam_20261111_030000_0.bag']
            for n in names:
                (Path(d)/n).write_bytes(b'x')
            for n in ('cam_20261111_014312.bag', 'cam_20261111_020000.bag'):
                got = [os.path.basename(p) for p in session_files(str(Path(d)/n))]
                self.assertEqual(got, [n])


class MapViewTests(unittest.TestCase):
    """当前帧的点必须落在累积图里同一个位置上 —— 两套坐标不能各算各的。"""

    PPM, HALF, SPAN, PIX = 20.0, 30.0, 12.0, 520

    def test_projection_matches_the_accumulated_canvas(self):
        pose = (3.0, -2.0, 0.4)
        side = self.SPAN*self.PPM
        height = self.HALF*2*self.PPM
        ox, oy = pose[0]-self.HALF, pose[1]-self.HALF
        crop_x = (pose[0]-ox)*self.PPM - side/2
        crop_y = (height-(pose[1]-oy)*self.PPM) - side/2
        for x, y in [(3.0, -2.0), (5.5, 1.0), (-1.0, 0.5), (3.0+5.9, -2.0)]:
            px, py = (x-ox)*self.PPM, height-(y-oy)*self.PPM
            want = ((px-crop_x)*self.PIX/side, (py-crop_y)*self.PIX/side)
            got = world_to_view(x, y, pose, self.SPAN, self.PIX)
            self.assertIsNotNone(got, (x, y))
            self.assertAlmostEqual(got[0], want[0], places=6)
            self.assertAlmostEqual(got[1], want[1], places=6)

    def test_robot_is_centred_and_up_is_positive_y(self):
        pose = (0.0, 0.0, 0.0)
        cx, cy = world_to_view(0.0, 0.0, pose, self.SPAN, self.PIX)
        self.assertAlmostEqual(cx, self.PIX/2.0, places=6)
        self.assertAlmostEqual(cy, self.PIX/2.0, places=6)
        ahead = world_to_view(0.0, 2.0, pose, self.SPAN, self.PIX)
        self.assertLess(ahead[1], cy)          # +y 在屏幕上是往上

    def test_points_outside_the_window_are_dropped(self):
        pose = (0.0, 0.0, 0.0)
        for x, y in [(7.0, 0.0), (0.0, -7.0), (6.5, 6.5)]:
            self.assertIsNone(world_to_view(x, y, pose, self.SPAN, self.PIX), (x, y))
        # 刚好在窗口里（±6 m 见方）的要留住
        self.assertIsNotNone(world_to_view(5.0, 5.0, pose, self.SPAN, self.PIX))


class CatalogTests(unittest.TestCase):
    def test_folder_first_with_legacy_and_active_session(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'cam_old_0.bag').write_bytes(b'old')
            (root/'session_a').mkdir();(root/'session_a'/'cam_x_0.bag').write_bytes(b'data')
            (root/'session_live').mkdir();(root/'session_live'/'cam_x_0.bag.active').touch()
            (root/'unrelated').mkdir()
            rows = sessions(str(root))
            self.assertEqual(len(rows),3)
            self.assertEqual(sum(x['active'] for x in rows),1)
            self.assertEqual(sum(x['bags'] for x in rows),2)


class SessionNamingTests(unittest.TestCase):
    def test_label_keeps_cjk_and_drops_path_characters(self):
        self.assertEqual(session_label('走廊建图'), '走廊建图')
        self.assertEqual(session_label('lab 11/10'), 'lab_11_10')
        self.assertEqual(session_label('../etc/passwd'), 'etc_passwd')
        self.assertEqual(session_label('  a  b  '), 'a_b')
        self.assertEqual(session_label('!!!'), '')
        self.assertEqual(session_label(None), '')

    def test_label_is_capped(self):
        self.assertEqual(len(session_label('x'*80)), 40)

    def test_empty_label_falls_back_to_bare_timestamp(self):
        self.assertEqual(session_dir_name('', '20261111_014312'), 'session_20261111_014312')
        self.assertEqual(session_dir_name('   ', '20261111_014312'), 'session_20261111_014312')
        self.assertEqual(session_dir_name('走廊', '20261111_014312'),
                         'session_20261111_014312_走廊')

    def test_rename_keeps_the_timestamp(self):
        # 只写标签 → 接在原来的时间戳后面，排序和来源都还在
        self.assertEqual(rename_target('session_20261111_014312_MZgKPg', '走廊建图'),
                         'session_20261111_014312_走廊建图')
        # 写完整名字 → 原样采用
        self.assertEqual(rename_target('session_20261111_014312_MZgKPg', 'validation_20261111_014312_lab'),
                         'validation_20261111_014312_lab')
        # 没有时间戳前缀的目录 → 补一个 session_，别把会话变成散包
        self.assertEqual(rename_target('legacy_stuff', 'lab'), 'session_lab')
        self.assertIsNone(rename_target('session_20261111_014312_x', '///'))


class StatusTests(unittest.TestCase):
    def test_custom_directory_and_current_session_only(self):
        with tempfile.TemporaryDirectory() as d:
            prefix = str(Path(d)/'cam_new.bag')
            current = Path(d)/'cam_new_0.bag.active'; current.write_bytes(b'live')
            (Path(d)/'cam_old_0.bag').write_bytes(b'history')
            s = snapshot({1: ['python2', '/opt/ros/melodic/bin/rosbag', 'record', '-O', prefix],
                          2: ['/opt/ros/melodic/lib/rosbag/record', '-O', prefix]})
            self.assertEqual(s['files'], [str(current)])
            self.assertEqual(s['directories'], [d])
            self.assertEqual(s['sessions'], 1)
            self.assertEqual(snapshot({3: ['bash', '/home/jetson/record_full.sh', '-o', d]})['state'], 'starting')


class GeometryTests(unittest.TestCase):
    def test_full_chain_and_reverse(self):
        tf = TransformIndex()
        tf.add('map', 'odom', 0, (2, 3, 0), True)
        tf.add('odom', 'base_footprint', 0, (1, 0, 0))
        tf.add('base_footprint', 'base_link', 0, (0, 0, 0), True)
        tf.add('base_link', 'laser', 0, (-.03, 0, math.pi), True)
        tf.finish()
        result = transform_xy(tf.lookup('map', 'laser', 0), 1, 0)
        self.assertAlmostEqual(result[0], 1.97)
        self.assertAlmostEqual(result[1], 3.)
        x, y = transform_xy(tf.lookup('laser', 'map', 0), *result)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)
        self.assertIsNone(tf.lookup('map', 'camera', 0))
        self.assertIsNone(tf.lookup('map', 'laser', 100))

    def test_yaw_interpolates_across_wrap(self):
        tf = TransformIndex()
        tf.add('odom', 'base', 0, (0, 0, math.radians(179)))
        tf.add('odom', 'base', 1, (1, 0, math.radians(-179)))
        tf.finish()
        x, _, yaw = tf.lookup('odom', 'base', .5)
        self.assertAlmostEqual(x, .5)
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_stale_map_association_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'cam_x_0.bag'; p.write_bytes(b'bag')
            meta = dict(sources=[dict(name=p.name, size=4, mtime=p.stat().st_mtime)],
                        yaml='map.yaml', trajectory='tf.bag')
            current=Path(map_manifest_path(str(p)));current.parent.mkdir(parents=True)
            current.write_text(json.dumps(meta))
            with self.assertRaises(ValueError): association(str(p))


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.src = self.root/'src'; self.src.mkdir()
        self.dst = self.root/'dst'; self.dst.mkdir()
        self.receive = self.dst/'legacy_src'; self.receive.mkdir()
        self.bin = self.root/'bin'; self.bin.mkdir()
        ssh = self.bin/'ssh'
        ssh.write_text('#!/usr/bin/env python3\nimport os,sys\nos.execv("/bin/sh",["sh","-c",sys.argv[-1]])\n')
        ssh.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ['PATH'])
        self.env.pop('ROBOT_PASS', None)
        self.data = b'abc123'*100000
        (self.src/'cam_test_0.bag').write_bytes(self.data)
        (self.src/'map_test.yaml').write_text('image: map.pgm\n')

    def tearDown(self): self.tmp.cleanup()

    def run_pull(self, *extra):
        return subprocess.run([sys.executable, str(ROOT/'recording/pull_session.py'),
                               '--robot', 'test', '-r', str(self.src), '-l', str(self.dst)]+list(extra),
                              env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def test_resume_verify_and_separate_delete(self):
        (self.receive/'cam_test_0.bag.part').write_bytes(self.data[:12345])
        p = self.run_pull()
        self.assertEqual(p.returncode, 0, p.stderr.decode())
        self.assertEqual((self.receive/'cam_test_0.bag').read_bytes(), self.data)
        self.assertTrue((self.src/'cam_test_0.bag').exists())
        self.assertTrue((self.receive/'map_test.yaml').exists())
        p = self.run_pull('-d')
        self.assertEqual(p.returncode, 0, p.stderr.decode())
        self.assertEqual(list(self.src.iterdir()), [])
        manifest = json.loads((self.receive/'transfer-manifest.json').read_text())
        self.assertEqual(len(manifest['deleted']), 2)

    def test_bad_destination_never_deletes_source(self):
        (self.receive/'cam_test_0.bag').write_bytes(b'corrupt')
        p = self.run_pull('-d')
        self.assertNotEqual(p.returncode, 0)
        self.assertEqual((self.receive/'cam_test_0.bag').read_bytes(), b'corrupt')
        self.assertTrue((self.src/'cam_test_0.bag').exists())

    def test_bad_partial_never_becomes_final(self):
        (self.receive/'cam_test_0.bag.part').write_bytes(b'wrong prefix')
        self.assertNotEqual(self.run_pull('-d').returncode, 0)
        self.assertFalse((self.receive/'cam_test_0.bag').exists())
        self.assertTrue((self.src/'cam_test_0.bag').exists())

    def test_active_recording_refused(self):
        (self.src/'cam_live.bag.active').touch()
        self.assertNotEqual(self.run_pull().returncode, 0)
        self.assertEqual(list(self.dst.glob('*.bag')), [])

    def test_preserves_independent_session_directories(self):
        for name,data in [('session_one',b'one'),('session_two',b'two')]:
            folder=self.src/name;folder.mkdir()
            (folder/'cam_test_0.bag').write_bytes(data)
        p=self.run_pull()
        self.assertEqual(p.returncode,0,p.stderr.decode())
        self.assertEqual((self.dst/'session_one'/'cam_test_0.bag').read_bytes(),b'one')
        self.assertEqual((self.dst/'session_two'/'cam_test_0.bag').read_bytes(),b'two')
        self.assertTrue((self.dst/'session_one'/'transfer-manifest.json').exists())

    def test_names_pulls_only_the_selected_bag(self):
        (self.src/'cam_other_0.bag').write_bytes(b'other'*1000)
        p=self.run_pull('--names','cam_test_0.bag')
        self.assertEqual(p.returncode,0,p.stderr.decode())
        self.assertTrue((self.receive/'cam_test_0.bag').exists())
        self.assertFalse((self.receive/'cam_other_0.bag').exists())

    def test_list_json_flags_a_missing_bag(self):
        p=self.run_pull('--list-json')
        self.assertEqual(p.returncode,0,p.stderr.decode())
        rows={r['name']:r for r in json.loads(p.stdout.decode())['files']}
        self.assertEqual(rows['cam_test_0.bag']['local'],'missing')
        self.assertFalse(rows['cam_test_0.bag']['recorded'])

    def test_list_json_reports_what_is_already_local(self):
        self.run_pull()
        p=self.run_pull('--list-json')
        self.assertEqual(p.returncode,0,p.stderr.decode())
        rows={r['name']:r for r in json.loads(p.stdout.decode())['files']}
        self.assertEqual(rows['cam_test_0.bag']['local'],'present')
        self.assertTrue(rows['cam_test_0.bag']['recorded'])
        self.assertEqual(rows['map_test.yaml']['local'],'present')

    def test_delete_only_keeps_what_has_no_local_copy(self):
        (self.src/'cam_other_0.bag').write_bytes(b'other'*1000)
        self.run_pull('--names','cam_test_0.bag')
        p=self.run_pull('--names','cam_test_0.bag','cam_other_0.bag','--delete-only')
        self.assertEqual(p.returncode,0,p.stderr.decode())
        self.assertFalse((self.src/'cam_test_0.bag').exists())
        self.assertTrue((self.src/'cam_other_0.bag').exists())

    def test_list_json_reports_a_partial_transfer(self):
        (self.receive/'cam_test_0.bag.part').write_bytes(self.data[:1000])
        p=self.run_pull('--list-json')
        self.assertEqual(p.returncode,0,p.stderr.decode())
        row={r['name']:r for r in json.loads(p.stdout.decode())['files']}['cam_test_0.bag']
        self.assertEqual(row['local'],'partial')
        self.assertEqual(row['partial_size'],1000)

    def test_delete_only_keeps_a_partial_copy(self):
        (self.receive/'cam_test_0.bag.part').write_bytes(self.data[:1000])
        p=self.run_pull('--names','cam_test_0.bag','--delete-only')
        self.assertNotEqual(p.returncode,0)
        self.assertTrue((self.src/'cam_test_0.bag').exists())
        self.assertTrue((self.receive/'cam_test_0.bag.part').exists())

    def test_delete_only_refuses_without_a_local_copy(self):
        p=self.run_pull('--names','cam_test_0.bag','--delete-only')
        self.assertNotEqual(p.returncode,0)
        self.assertTrue((self.src/'cam_test_0.bag').exists())


class LifecycleTests(unittest.TestCase):
    def test_completed_split_does_not_hide_active_tail(self):
        for incomplete in (False, True):
            with tempfile.TemporaryDirectory() as d:
                root = Path(d); (root/'out').mkdir(); (root/'bin').mkdir()
                script = (ROOT/'recording/auto_record_cam.sh').read_text()
                script = script.replace('source /opt/ros/melodic/setup.bash 2>/dev/null || true', ':')
                script = script.replace('"$HOME/software/', '"/nonexistent/software/').replace('"$HOME/transbot_ws/', '"/nonexistent/transbot_ws/')
                (root/'auto.sh').write_text(script)
                (root/'calibration_schema.py').write_bytes((ROOT/'calibration_schema.py').read_bytes())
                for helper in ['session_finalize.py', 'record_quality.py', 'record_guard.py','live_health.py','stream_health.py','clock_check.py']:
                    (root/helper).write_bytes((ROOT/'recording'/helper).read_bytes())
                for name, body in {
                    'rostopic': '#!/bin/sh\n[ "$1" = list ] && printf "/scan\\n/odom\\n/tf\\n"; exit 0\n',
                    'rosnode': '#!/bin/sh\nexit 0\n',
                    'rosbag': '#!/usr/bin/env python3\nimport sys,pathlib\nif sys.argv[1]=="record":\n p=pathlib.Path(sys.argv[sys.argv.index("-O")+1]); p.with_name(p.stem+"_0.bag").write_bytes(b"ok")\n '+('p.with_name(p.stem+"_1.bag.active").write_bytes(b"unfinished")' if incomplete else 'pass')+'\n',
                }.items():
                    p = root/'bin'/name; p.write_text(body); p.chmod(0o755)
                env = dict(os.environ, PATH=str(root/'bin')+os.pathsep+os.environ['PATH'],
                           MIN_FREE_GB='0', ROS_IP='127.0.0.1',ROS_MASTER_URI='http://127.0.0.1:1')
                p = subprocess.run(['bash', str(root/'auto.sh'), '-p', 'nav', '-d', '1', '-o', str(root/'out')],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=45)
                if incomplete:
                    self.assertNotEqual(p.returncode, 0)
                else:
                    # Closed bytes are not a valid bag with required sensor data.
                    self.assertNotEqual(p.returncode, 0)
                    p2 = subprocess.run(['bash', str(root/'auto.sh'), '-p', 'nav', '-d', '1', '-o', str(root/'out')],
                                        env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=45)
                    self.assertNotEqual(p2.returncode,0)
                    folders=list((root/'out').glob('session_*'))
                    self.assertEqual(len(folders),2)
                    self.assertTrue(all(list(f.glob('*.bag')) for f in folders))

    def test_atomic_lock_and_cancel_during_startup(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            script = (ROOT/'recording/record_full.sh').read_text()
            # Only replace hardware/environment dependencies and global lock paths.
            script = script.replace('/tmp/transbot_record.', d+'/record.')
            script = script.replace('source /opt/ros/melodic/setup.bash', ':')
            script = script.replace('if [ -f "$HOME/', 'if [ -f "/nonexistent/')
            fake = root/'auto_record_cam.sh'
            fake.write_text('#!/bin/bash\ntrap "echo cancelled > '+d+'/cancelled; exit 130" TERM INT\nwhile :; do sleep .1; done\n')
            fake.chmod(0o755)
            (root/'preflight.sh').write_text('#!/bin/bash\nexit 0\n')
            pth = root/'record_full.sh'; pth.write_text(script); pth.chmod(0o755)
            p = subprocess.Popen(['bash', str(pth), '--no-launch', '-o', str(root/'out')], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic()+5
                while not (root/'record.pid').exists() and time.monotonic() < deadline:
                    time.sleep(.05)
                second = subprocess.run(['bash', str(pth), '--no-launch', '-o', str(root/'out')], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.assertEqual(second.returncode, 1)
                time.sleep(.3)
                p.terminate(); p.wait(timeout=5)
                self.assertTrue((root/'cancelled').exists())
                self.assertFalse((root/'record.pid').exists())
            finally:
                if p.poll() is None: p.terminate(); p.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
