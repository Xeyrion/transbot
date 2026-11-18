"""Run explicitly inside xvfb: BAGTOOLS_GUI_TEST=1 python3 -m unittest discover -s tests."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
import gc
from unittest.mock import patch,Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'recording'))


@unittest.skipUnless(os.environ.get('BAGTOOLS_GUI_TEST')=='1','requires isolated Xvfb display')
class GuiTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        import bag_quality_gui as gui
        self.gui=gui;self.root=tk.Tk()
        self.tmp=tempfile.TemporaryDirectory();self.folder=Path(self.tmp.name)
        self.archive_env=patch.dict(os.environ,{'BAGTOOLS_ARCHIVE_DIR':str(self.folder/'archive')});self.archive_env.start()
        self.old_server=gui.SERVER;self.old_probe=gui.PROBE

    def tearDown(self):
        self.gui.SERVER=self.old_server;self.gui.PROBE=self.old_probe
        try:self.root.destroy()
        except Exception:pass
        self.root=None
        gc.collect()  # Dispose Tcl interpreters on the UI thread, before another test starts workers.
        self.tmp.cleanup()
        self.archive_env.stop()

    def pump(self, predicate, seconds=5):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.root.update();time.sleep(.005)
            if predicate():return
        self.fail('UI condition timed out')

    def test_immediate_window_loading_then_ready(self):
        server=self.folder/'server.py'
        server.write_text('''import json,sys,time
time.sleep(.75)
print(json.dumps(dict(dur=5,world='base',has_map=False)));sys.stdout.flush()
while True:
 line=sys.stdin.readline()
 if not line:break
 q=json.loads(line)
 if q.get('cmd')=='close':break
 print(json.dumps(dict(t=q['t'],frame='base',pose=[0,0,0],err=None)));sys.stdout.flush()
''')
        self.gui.SERVER=str(server)
        start=time.monotonic();v=self.gui.BagViewer(self.root,str(self.folder),'unused.bag')
        self.assertLess(time.monotonic()-start,.5)
        self.root.update()
        self.assertTrue(v.winfo_ismapped())
        self.assertTrue(v._loading)
        self.assertTrue(v.loading_frame.winfo_ismapped())
        self.pump(lambda:not v._loading)
        self.assertFalse(v.loading_frame.winfo_ismapped())
        self.assertIn('5.0',v.var_t.get())
        v.close()

    def test_failure_stops_spinner(self):
        server=self.folder/'broken.py';server.write_text("raise RuntimeError('test reader failure')\n")
        self.gui.SERVER=str(server)
        v=self.gui.BagViewer(self.root,str(self.folder),'unused.bag')
        self.pump(lambda:v._load_error)
        self.assertIn('test reader failure',v.var_loading.get())
        self.assertFalse(v._loading)
        v.close()

    def test_semantic_selector_boxes_map_and_export(self):
        from PIL import Image
        image = self.folder/'sample.png';Image.new('RGB', (100, 80)).save(str(image))
        server = self.folder/'semantic_server.py'
        server.write_text('''import json,sys
context=dict(dur=5,t0=100,world='map',has_map=True)
print(json.dumps(dict(dur=5,t0=100,world='base',has_map=False,
    semantic_runs=[dict(path='/semantic/test',label='yolo26s',preset='yolo26s'),dict(path='/semantic/old',label='C1')])));sys.stdout.flush()
previous=None
while True:
 line=sys.stdin.readline()
 if not line:break
 q=json.loads(line)
 if q.get('cmd')=='close':break
 selected=q.get('semantic_run')
 out=dict(t=q['t'],record_time=100+q['t'],frame='map' if selected else 'base',
     pose=[0,0,0],scan=[[1,0]],rgb=%r,depth=%r,stamps={},err=None,semantic_run=selected)
 if previous!=selected:
  out['context']=context if selected else dict(dur=5,t0=100,world='base',has_map=False)
 if selected:
  out['semantic']=dict(model='yolo26s',sample='0000',world='map',detections=[
    dict(label='chair',score=.9,bbox_xywh=[5,20,20,20],status='fused',projection='accepted',object_number=1,included=True),
    dict(label='backpack',score=.8,bbox_xywh=[55,40,20,20],status='excluded',projection='accepted',included=False)],
    objects=[dict(label='chair',number=1,x=1,y=1)])
 previous=selected
 print(json.dumps(out));sys.stdout.flush()
''' % (str(image), str(image)))
        self.gui.SERVER = str(server)
        (self.folder/'unused.bag').touch()
        v = self.gui.BagViewer(self.root, str(self.folder), 'unused.bag')
        self.pump(lambda:not v._loading)
        self.assertEqual(tuple(v.semantic_picker['values'])[1:], ('YOLO26s',))
        v.var_all_runs.set(True);v._refresh_semantic_choices()
        self.assertIn('C1', v.semantic_picker['values'])
        v.var_all_runs.set(False);v._refresh_semantic_choices()
        v.semantic_picker.current(1);v._select_semantic()
        self.pump(lambda:bool(v._last.get('semantic')))
        self.assertEqual(v.world, 'map')
        self.assertEqual(v._export_rgb.getpixel((5, 20)), self.gui.COLORS['fused'])
        self.assertIsNotNone(v._map_panel)
        self.assertEqual(v._export_rgb.getpixel((55, 55)), (0, 0, 0))
        v.var_boxes.set(False)
        v.var_other_classes.set(True)
        v._toggle_other_classes()
        self.assertTrue(v.var_boxes.get())
        self.assertEqual(v._export_rgb.getpixel((55, 55)), self.gui.COLORS['excluded'])
        self.assertIn(self.gui.tr('本帧显示 %d 个其它类别框。') % 1, v.var_semantic_note.get())
        v._update_semantic_note(dict(v._last['semantic'], detections=[]))
        self.assertIn(self.gui.tr('本帧没有其它类别的检测框。'), v.var_semantic_note.get())
        v.var_other_classes.set(False)
        v._toggle_other_classes()
        self.assertEqual(v._export_rgb.getpixel((55, 55)), (0, 0, 0))
        with patch.object(self.gui,'draw_detections',wraps=self.gui.draw_detections) as draw:
            v._apply(dict(v._last))
            draw.assert_not_called()  # Same semantic sample while the timeline advances.
        marked = v._map_panel.tobytes()
        v.var_points.set(False);v._redraw_semantic()
        self.assertNotEqual(marked, v._map_panel.tobytes())
        v.var_boxes.set(False);v._redraw_semantic()
        self.assertEqual(v._export_rgb.getpixel((5, 20)), (0, 0, 0))
        v.var_boxes.set(True);v.var_points.set(True);v._redraw_semantic();v.export_combo()
        self.assertEqual(len(list((self.folder/'exports').glob('*_yolo26s.png'))), 1)
        # Runs using the same model and time must not overwrite each other's exports.
        v._last['semantic']['run_id']='trial_B1';v.export_combo()
        v._last['semantic']['run_id']='trial_C2';v.export_combo()
        self.assertEqual(len(list((self.folder/'exports').glob('*_trial_*.png'))),2)
        initial_width=v.cv.winfo_width()
        self.assertGreater(v.body.sashpos(0),100,'initial RGB pane must remain visible')
        v.set_layout('map')
        self.pump(lambda:v.cv.winfo_width()>initial_width+80)
        self.pump(lambda:v._ph_rgb.width()==v.lbl_rgb.winfo_width())
        photo=v._ph_rgb;v._redraw_view();self.assertIs(photo,v._ph_rgb)
        v.open_map_window()
        self.pump(lambda:v._ph_map_large is not None)
        self.assertGreater(v._large_canvas.winfo_width(),v.cv.winfo_width())
        span=v._viewports[v.cv].span;centre=v._viewports[v.cv].centre
        v.zoom_map(.8)
        self.assertAlmostEqual(v._viewports[v.cv].span,span*.8)
        self.assertEqual(v._viewports[v.cv].centre,centre)
        v.overview_map();self.assertAlmostEqual(v._viewports[v.cv].span,span)
        v._close_map_window()
        with patch.object(v,'_open_path') as opened:
            v.open_results();opened.assert_called_once_with('/semantic/test')
        with patch.object(self.gui,'HERE',str(self.folder)),patch.object(v,'_open_path') as opened:
            v.open_gallery()
            opened.assert_called_once_with(str(self.folder/'semantic'/'unused'/'gallery'))
        def export_saved(folder,gallery):
            self.assertEqual(folder,'/semantic/test');time.sleep(.15)
            self.assertEqual(gallery,str(self.folder/'semantic'/'unused'/'gallery'))
            return str(self.folder/'final_semantic_map.png')
        with patch.object(self.gui,'export_recording_map',side_effect=export_saved),patch.object(v,'_open_path') as opened:
            start=time.monotonic();v.open_final_map();self.assertLess(time.monotonic()-start,.1)
            self.pump(lambda:opened.called)
            opened.assert_called_once_with(str(self.folder/'final_semantic_map.png'))
        v.semantic_picker.current(0);v._select_semantic()
        self.pump(lambda:not v._last.get('semantic'))
        self.assertEqual(v.world, 'base')
        self.assertEqual(v._export_rgb.getpixel((5, 20)), (0, 0, 0))
        def start_generation(command, **kwargs):
            self.assertEqual(command[4:], ['--preset', 'yolo26s-seg'])
            output = Path(command[3]);output.mkdir()
            self.assertEqual(output.parent, self.folder/'semantic'/'unused')
            (output/'run.json').write_text('{"state":"complete","world_frame":"odom"}')
            scan.return_value = [dict(path=str(output), label='YOLO26s Segmentation result', preset='yolo26s-seg')]
            process = Mock()
            process.poll.side_effect = [None, 0]
            return process
        with patch.object(self.gui,'HERE',str(self.folder)), \
                patch.object(self.gui,'discover_runs') as scan, \
                patch.object(self.gui.subprocess,'Popen',side_effect=start_generation) as launch:
            menu = v.nametowidget(str(v.generate_button['menu']))
            menu.invoke(1)
            launch.assert_called_once()
            self.assertTrue(v.generate_button.instate(['disabled']))
            self.pump(lambda:v._semantic_proc is None)
            self.assertEqual(v.semantic_picker.get(), self.gui.SEMANTIC_PRESETS['yolo26s-seg'])
            self.assertIn(self.gui.tr('此结果使用里程计坐标（odom），位置可能随里程计累计漂移。'), v.var_generation.get())
            self.pump(lambda:v._last.get('semantic_run') == v._semantic_output)
        v.close()

    def test_session_selection_and_nonblocking_preview(self):
        session=self.folder/'session_test';session.mkdir();(session/'cam_test_0.bag').write_bytes(b'fake')
        probe=self.folder/'probe.py'
        probe.write_text("import sys,time\ntime.sleep(.4)\nopen(sys.argv[sys.argv.index('--frame')+1],'wb').write(b'P6\\n1 1\\n255\\n'+b'\\0\\0\\0')\n")
        self.gui.PROBE=str(probe)
        app=self.gui.App(self.root,str(self.folder));self.root.update()
        self.assertEqual(app.mode,'sessions');self.assertFalse(app.body.winfo_ismapped())
        app.catalog.selection_set(app.catalog.get_children()[0])
        with patch.object(self.gui,'BagViewer') as viewer:
            self.root.nametowidget(str(app.btn_semantic['menu'])).invoke(1)
            viewer.assert_called_once_with(self.root,str(session),'cam_test_0.bag',preset='yolo26s-seg')
        app.catalog.selection_set(app.catalog.get_children()[0]);app.open_session();self.root.update()
        self.assertEqual(app.mode,'bags');self.assertEqual(len(app.tree.get_children()),1)
        start=time.monotonic();app._show_preview('cam_test_0.bag')
        self.assertLess(time.monotonic()-start,.1)
        ticks=[];self.root.after(30,lambda:ticks.append(True))
        self.pump(lambda:bool(ticks));self.assertFalse(app._preview_job.done())
        self.pump(lambda:app._photo is not None)
        app.back_sessions();self.root.update();self.assertEqual(app.mode,'sessions')
        app.close()

    def test_session_rename_keeps_timestamp(self):
        session = self.folder/'session_20261111_014312_MZgKPg'
        session.mkdir(); (session/'cam_20261111_014312_0.bag').write_bytes(b'fake')
        app = self.gui.App(self.root, str(self.folder)); self.root.update()
        app.catalog.selection_set(app.catalog.get_children()[0])
        with patch.object(self.gui.simpledialog, 'askstring', return_value='走廊建图'):
            app.rename_session()
        self.root.update()
        renamed = self.folder/'session_20261111_014312_走廊建图'
        self.assertTrue(renamed.is_dir(), 'timestamp prefix must survive a rename')
        self.assertFalse(session.exists())
        self.assertEqual(app.catalog.item(app.catalog.get_children()[0], 'values')[0],
                         'session_20261111_014312_走廊建图')
        app.close()

    def test_mapping_decline_keeps_folder_unchanged(self):
        session=self.folder/'session_test';session.mkdir();(session/'cam_20261115_120000_0.bag').write_bytes(b'bag')
        from viewer_core import map_manifest_path
        pointer=Path(map_manifest_path(str(session/'cam_20261115_120000_0.bag')));pointer.parent.mkdir(parents=True);pointer.write_text('{}')
        app=self.gui.App(self.root,str(self.folder));self.root.update()
        app.catalog.selection_set(app.catalog.get_children()[0])
        with patch.object(self.gui.messagebox,'askyesno',return_value=False) as ask, \
             patch.object(self.gui,'new_job') as create:
            app.build_map()
        ask.assert_called_once();create.assert_not_called()
        self.assertIsNone(app._mapping_proc);self.assertFalse((session/'.bagtools_maps').exists())
        app.close()

    def test_mapping_window_is_nonblocking_and_cancel_is_a_request(self):
        import mapping_job
        session=self.folder/'session_test';session.mkdir();bag=session/'cam_20261115_120000_0.bag';bag.write_bytes(b'bag')
        app=self.gui.App(self.root,str(self.folder));self.root.update()
        app.catalog.selection_set(app.catalog.get_children()[0])
        child=Mock();child.poll.return_value=None
        with patch.object(self.gui.subprocess,'Popen',return_value=child):
            start=time.monotonic();app.build_map()
            self.assertLess(time.monotonic()-start,.5)
        self.root.update();self.assertTrue(app._mapping_window.winfo_ismapped());self.assertTrue(app.busy)
        app.cancel_mapping();self.assertTrue((app._mapping_job/'cancel').exists())
        child.terminate.assert_not_called()
        mapping_job.atomic_json(app._mapping_job/'status.json',dict(state='cancelled',stage='cancelled'))
        child.poll.return_value=130;app._mapping_next_poll=0;app._poll_mapping()
        self.assertFalse(app.busy);self.assertIsNone(app._mapping_proc)
        app.cancel_mapping();app.close()

    def test_rename_refuses_live_session_and_root(self):
        live = self.folder/'session_20261111_014312_live'
        live.mkdir(); (live/'cam_20261111_014312_0.bag.active').touch()
        app = self.gui.App(self.root, str(self.folder)); self.root.update()
        app.catalog.selection_set(app.catalog.get_children()[0])
        with patch.object(self.gui.messagebox, 'showinfo') as info, \
             patch.object(self.gui.simpledialog, 'askstring') as ask:
            app.rename_session()
        info.assert_called_once()
        ask.assert_not_called()
        self.assertTrue(live.is_dir())
        app.close()

    def test_start_args_carry_the_session_name(self):
        import record_gui
        self.assertEqual(record_gui.start_args(300, '/home/jetson/bags'),
                         ['start', '-d', '300', '-o', '/home/jetson/bags'])
        self.assertEqual(record_gui.start_args(60, '/b', '走廊建图'),
                         ['start', '-d', '60', '-o', '/b', '-n', '走廊建图'])

    def test_recorder_idle_rates_survive_wall_clock_adjustment(self):
        import record_gui
        with patch('record_gui.threading.Thread.start'):
            app=record_gui.App(self.root,record_gui.Remote('jetson@192.168.1.11','unused'))
        app.state=dict(rec=0,ros_ok=1,health_state='not_recording',rgb=30.,depth=29.,scan=7.5,
                       master=1,driver=1,disk_free_gb=23.)
        app.state_t=time.monotonic()
        with patch('record_gui.time.time',return_value=0.):app._render()
        self.assertEqual(app.v_src['rgb'].cget('text'),'30.0 Hz')
        self.assertEqual(app.v_src['scan'].cget('text'),'7.5 Hz')
        app.state['scan']=0.;app._render()
        self.assertEqual(app.v_src['scan'].cget('text'),'0.0 Hz')
        app.state_t-=5.;app._render()
        self.assertEqual(app.v_src['rgb'].cget('text'),'--')
        app.closing=True

    def test_workbench_launches_without_folder_navigation(self):
        from workbench import Workbench
        with patch.dict(os.environ,{'BAGTOOLS_CACHE_DIR':str(self.folder)}):
            home=Workbench(self.root,str(self.folder/'data'))
            child=Mock();child.poll.return_value=None
            with patch('workbench.subprocess.Popen',return_value=child) as spawn:
                home.open_tool('record')
                home.open_tool('record')
                self.assertEqual(spawn.call_count,1)
                self.assertEqual(spawn.call_args[1]['env']['LOCAL_DIR'],str(self.folder/'data'))
                home.close()
                child.terminate.assert_not_called()

    def test_preparing_state_blocks_duplicate_start_and_maintenance(self):
        import record_gui
        with patch('record_gui.threading.Thread.start'):
            app=record_gui.App(self.root,record_gui.Remote('jetson@192.168.1.11','unused'))
        app.state=dict(rec=0,phase='preparing',ros_ok=1,rgb=30.,depth=30.,scan=7.)
        app.state_t=time.monotonic();app.online=True;app._render()
        self.assertEqual(app.btn_start.cget('state'),'disabled')
        self.assertTrue(all(b.cget('state')=='disabled' for b in app.prepare_buttons))
        with patch.object(app,'_run_script') as run:
            app.on_prepare('clock');run.assert_not_called()
            app.state['phase']='idle';app._render()
            app.on_prepare('camera');self.assertEqual(run.call_args[0][0],['prepare','camera'])
        app.closing=True

    def test_english_workbench_and_recorder_labels(self):
        from workbench import Workbench
        with patch.dict(os.environ,{'BAGTOOLS_LANG':'en'}):
            home=Workbench(self.root,str(self.folder/'data'))
            self.root.update()
            self.assertEqual(home.buttons['record'].cget('text'),'Recording Console')
            self.assertEqual(home.buttons['review'].cget('text'),'Session Browser')
            self.assertEqual(self.root.title(),'Transbot Data Workbench')
            home.close()


if __name__=='__main__':unittest.main()
