#!/usr/bin/env python3
"""Offline gmapping in a private ROS master; exports map, TF and source association."""
import argparse
import glob
import json
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
import pathlib
import sys

from viewer_core import session_files


def run_child(command):
    """Give nested mappers time to finalize their private ROS processes on cancel."""
    child=subprocess.Popen(command,start_new_session=True)
    try:
        code=child.wait()
        if code:raise subprocess.CalledProcessError(code,command)
    except KeyboardInterrupt:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=240)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        raise


def skip(reason):
    """数据本身不需要/不可能建图 —— 退出码 3，和真失败区分开。

    调用方（build_maps.sh）靠退出码分类，不靠解析输出里的字串：回溯会把检查
    脚本源码原样打出来，按字串匹配会误判。
    """
    print(reason, flush=True)
    raise SystemExit(3)


def probe_bags(sources, env):
    """一次性问清三个问题：有没有 /scan、有没有 /map、有没有 map->odom。"""
    code = '''import json,rosbag,sys
res={'scan':0,'map':0,'map_odom':False,'duration':0}
for p in sys.argv[1:]:
 b=rosbag.Bag(p)
 res['duration']+=b.get_end_time()-b.get_start_time()
 info=b.get_type_and_topic_info()[1]
 if info.get('/scan'): res['scan']+=info.get('/scan')[1]
 if info.get('/map'): res['map']+=info.get('/map')[1]
 if not res['map_odom']:
  for topic,m,t in b.read_messages(topics=['/tf','/tf_static']):
   if any(x.header.frame_id.strip('/')=='map' and x.child_frame_id.strip('/')=='odom' for x in m.transforms):
    res['map_odom']=True
    break
 b.close()
print(json.dumps(res))
'''
    done = subprocess.run(['python2', '-c', code]+sources, env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    text = (done.stdout or b'').decode('utf-8', 'replace').strip()
    if done.returncode != 0:
        raise RuntimeError((done.stderr or b'').decode('utf-8', 'replace').strip()[-200:] or
                           'bag probe failed')
    try:
        return json.loads(text.splitlines()[-1])
    except (ValueError, IndexError):
        raise RuntimeError('bag probe returned nothing: ' + text[:160])


def main():
    def cancel(_signal,_frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,cancel)
    signal.signal(signal.SIGINT,cancel)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag', help='any split in the source session')
    ap.add_argument('--rate', type=float, default=2.)
    ap.add_argument('--port', type=int, default=11329)
    ap.add_argument('--output-dir',help='NEW isolated output directory; source bags remain untouched')
    ap.add_argument('--recompute',action='store_true',help='Explicitly strip old map->odom and rebuild a NEW coordinate solution')
    ap.add_argument('--calibration'); ap.add_argument('--allow-estimated-extrinsics',action='store_true')
    ap.add_argument('--override-recorded-extrinsics',action='store_true')
    ap.add_argument('--record-start',type=float); ap.add_argument('--record-end',type=float)
    ap.add_argument('--allow-gaps',action='store_true')
    args = ap.parse_args()
    if args.rate <= 0:
        ap.error('--rate must be positive')
    sources = session_files(args.bag)
    if not sources: ap.error('No source bags')
    if args.recompute and not args.output_dir: ap.error('--recompute requires a NEW --output-dir')
    if args.output_dir:
        target=pathlib.Path(args.output_dir).resolve(); target.mkdir(parents=True)
        if args.recompute:
            helper=os.path.join(os.path.dirname(__file__),'semantic','prepare_navigation.py')
            cmd=['python2',helper,sources[0],str(target/'navigation')]
            for key,flag in [('calibration','--calibration'),('record_start','--record-start'),('record_end','--record-end')]:
                if getattr(args,key) is not None: cmd += [flag,str(getattr(args,key))]
            for key in ['allow_estimated_extrinsics','override_recorded_extrinsics','allow_gaps']:
                if getattr(args,key): cmd.append('--'+key.replace('_','-'))
            run_child(cmd)
            repaired=target/'navigation'/os.path.basename(sources[0])
            run_child([sys.executable,__file__,str(repaired),'--rate',str(args.rate),'--port',str(args.port)])
            with open(str(repaired)+'.map.json') as f: linked=json.load(f)
            for name in ['yaml','trajectory']:
                if linked.get(name): linked[name]=str((repaired.parent/linked[name]).resolve())
            with open(str(target/'navigation'/'repair.json')) as f: repair=json.load(f)
            linked.update(replace_map_odom=True,replace_installation_edges=repair['replaced_installation_edges'],
                record_bounds=repair['record_bounds'],repair=repair)
        else:
            for source in sources: (target/os.path.basename(source)).symlink_to(os.path.abspath(source))
            first=target/os.path.basename(sources[0])
            run_child([sys.executable,__file__,str(first),'--rate',str(args.rate),'--port',str(args.port)])
            if os.path.isfile(str(first)+'.map.json'):
                with open(str(first)+'.map.json') as f: linked=json.load(f)
                for name in ['yaml','trajectory']:
                    if linked.get(name): linked[name]=str((target/linked[name]).resolve())
            else:
                maps=sorted(target.glob('map_*.yaml'))
                if not maps: raise RuntimeError('No exported map')
                linked=dict(yaml=str(maps[-1]),trajectory=None)
        linked['sources']=[dict(name=os.path.basename(p),size=os.path.getsize(p),mtime=os.path.getmtime(p)) for p in sources]
        for key in ['yaml','trajectory']:
            if linked.get(key): linked[key]=os.path.relpath(linked[key],str(target))
        with open(str(target/'association.json'),'w') as f: json.dump(linked,f,indent=2)
        print('Map association: '+str(target/'association.json')); return
    # Never bind over an existing master, and never use the robot's ROS master.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
               ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    facts = probe_bags(sources, env)
    if facts['map'] and facts['map_odom']:
        # 地图本来就在包里，抄出来就行 —— 重跑 gmapping 只会和 map->odom 打架
        subprocess.run(['python2', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                'extract_map.py')] + sources, env=env, check=True)
        return
    if facts['map_odom']:
        skip('包里已有 map->odom，但没有 /map 可以导出；重建会和已有变换冲突')
    if not facts['scan']:
        skip('包里没有 /scan，建不了图')
    stem = sources[0]+'.mapping'
    if glob.glob(stem+'*') or os.path.exists(sources[0]+'.map.json'):
        raise RuntimeError('Mapping output exists; preserve/rename it before rebuilding')
    output = stem+'.bag'
    children = []
    # 日志等检查过了再开：拒绝重建时不该在会话目录里留下一个空 .log，
    # 否则下次批量建图会把它误当成"已有地图产物"而永久跳过。
    log = None

    def start(cmd):
        p = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        children.append(p)
        return p

    def run(cmd, timeout=60):
        subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=timeout)

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:p.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)

    try:
        log = open(stem+'.log', 'w')
        master = start(['roscore', '-p', str(args.port)])
        deadline = time.monotonic()+20
        while True:
            try:
                xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/offline_mapping')
                break
            except OSError:
                if master.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Private ROS master failed to start')
                time.sleep(.2)
        run(['rosparam', 'set', '/use_sim_time', 'true'])
        mapper = start(['rosrun', 'gmapping', 'slam_gmapping', '__name:=offline_gmapping',
                        '_base_frame:=base_footprint', '_odom_frame:=odom', '_map_frame:=map',
                        '_map_update_interval:=1.0', '_delta:=0.05', '_particles:=60',
                        '_xmin:=-20', '_xmax:=20', '_ymin:=-20', '_ymax:=20', 'scan:=/scan'])
        recorder = start(['rosrun', 'rosbag', 'record', '--lz4', '-O', output,
                          '/tf', '/tf_static', '/map', '/map_metadata'])
        time.sleep(3)
        if mapper.poll() is not None or recorder.poll() is not None:
            raise RuntimeError('Mapper or TF recorder failed; inspect mapping log')
        # Explicit topic allowlist excludes /cmd_vel and every actuator topic.
        player = start(['rosbag', 'play', '--clock', '--rate', str(args.rate),
                        '--bags']+sources+['--topics', '/scan', '/odom', '/tf', '/tf_static'])
        if player.wait(timeout=max(60,facts['duration']/args.rate+120)) != 0:
            raise RuntimeError('Bag playback failed')
        # 回放结束后 /clock 就没了，但 /use_sim_time 还是 true —— 这时候起的
        # 节点会卡在 "no clock published. Still waiting for valid time" 上，
        # map_saver 会一直等到超时，地图存不下来。切回真实时间再存图。
        run(['rosparam', 'set', '/use_sim_time', 'false'])
        run(['rosrun', 'map_server', 'map_saver', '-f', stem], timeout=30)
        stop(recorder)
        if not os.path.isfile(output):
            raise RuntimeError('TF recording did not finalize')
        # Make the raster portable and associate it only with this exact session.
        import yaml
        with open(stem+'.yaml') as f:
            data = yaml.safe_load(f)
        data['image'] = os.path.basename(stem+'.pgm')
        with open(stem+'.yaml', 'w') as f:
            yaml.safe_dump(data, f)
        manifest = dict(yaml=os.path.basename(stem+'.yaml'), trajectory=os.path.basename(output),
                        sources=[dict(name=os.path.basename(p), size=os.path.getsize(p),
                                      mtime=os.path.getmtime(p)) for p in sources])
        with open(sources[0]+'.map.json.tmp', 'w') as f:
            json.dump(manifest, f, indent=2)
        os.replace(sources[0]+'.map.json.tmp', sources[0]+'.map.json')
        print('地图和轨迹已关联：'+sources[0]+'.map.json', flush=True)
    finally:
        cleanup_errors=[]
        for p in reversed(children):
            try: stop(p)
            except Exception as exc: cleanup_errors.append(str(exc))
        if log is not None:
            log.close()
        if cleanup_errors: raise RuntimeError('Some offline processes did not finalize: '+repr(cleanup_errors))


if __name__ == '__main__':
    main()
