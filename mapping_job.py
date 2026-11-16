#!/usr/bin/env python3
"""Transactional session mapping: new outputs first, switch the active map only on success."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from viewer_core import session_files,map_directory,map_manifest_path

HERE=Path(__file__).resolve().parent


def recording_groups(folder):
    groups={}
    for path in sorted(Path(folder).glob('*.bag')):
        if path.name.endswith('.mapping.bag'):continue
        files=session_files(str(path))
        if files:groups[files[0]]=files
    return [groups[k] for k in sorted(groups)]


def map_artifacts(bag):
    files=session_files(str(bag))
    if not files:return []
    candidates=[Path(map_manifest_path(files[0])),Path(map_directory(files[0]))/'map.yaml']
    return [str(p) for p in candidates if p.is_file()]


def archive_root():
    return Path(os.environ.get('BAGTOOLS_ARCHIVE_DIR',str(HERE/'archive'))).expanduser().resolve()


def new_job(bag):
    files=session_files(str(bag))
    if not files:raise ValueError('No source bags')
    parent=Path(files[0]).parent
    if list(parent.glob('*.bag.active')):raise ValueError('Recording is still active')
    runs=archive_root()/'builds'/parent.name/Path(map_directory(files[0])).name
    runs.mkdir(parents=True,exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('run_%Y%m%d_%H%M%S_'),dir=str(runs)))


def read_status(job):
    try:return json.loads((Path(job)/'status.json').read_text())
    except (OSError,ValueError):return {}


def atomic_json(path,data):
    temporary=Path(str(path)+'.tmp')
    with temporary.open('w') as f:
        json.dump(data,f,indent=2);f.flush();os.fsync(f.fileno())
    os.replace(str(temporary),str(path))


def source_snapshot(files):
    return [dict(name=Path(p).name,size=Path(p).stat().st_size,mtime=Path(p).stat().st_mtime) for p in files]


def publish(bag,job,expected_sources,previous):
    """Pointer replacement is the only commit; failed jobs never disturb an old map."""
    files=session_files(str(bag));job=Path(job);pointer=Path(map_manifest_path(files[0]))
    if source_snapshot(files)!=expected_sources:raise ValueError('Source bags changed during mapping')
    if list(Path(files[0]).parent.glob('*.bag.active')):raise ValueError('Recording is still active')
    current=pointer.read_bytes() if pointer.exists() else None
    if current!=previous:raise ValueError('Active map changed during mapping; refusing to replace it')
    with (job/'result/association.json').open() as f:data=json.load(f)
    for field in ('yaml','trajectory'):
        target=(job/'result'/data[field]).resolve()
        if not target.is_file() or (job.resolve() not in target.parents):raise ValueError('Missing mapping output: '+field)
        data[field]=str(target)
    import yaml
    from PIL import Image
    yamlpath=Path(data['yaml']);info=yaml.safe_load(yamlpath.read_text())
    image=Path(info['image']);image=image if image.is_absolute() else yamlpath.parent/image
    if not image.is_file() or image.resolve().parent!=yamlpath.resolve().parent:raise ValueError('Missing map raster')
    with Image.open(str(image)) as raster:
        pixels=raster.convert('L').tobytes()
        if not any(v<90 for v in pixels) or not any(v>205 for v in pixels):raise ValueError('Map is empty')
        data['map_dimensions']=list(raster.size)
    data.update(sources=expected_sources,map_origin='recomputed_from_session',build_id=job.name)
    if previous is not None:(job/'previous_association.json').write_bytes(previous)
    if (job/'cancel').exists():raise KeyboardInterrupt()
    current=pointer.parent;current.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.building-',dir=str(current.parent)) as temp:
        candidate=Path(temp)/'map';candidate.mkdir()
        shutil.copy2(str(image),str(candidate/'map.pgm'))
        shutil.copy2(data['trajectory'],str(candidate/'trajectory.bag'))
        info['image']='map.pgm'
        with (candidate/'map.yaml').open('w') as f:yaml.safe_dump(info,f)
        data.update(yaml='map.yaml',trajectory='trajectory.bag')
        atomic_json(candidate/'association.json',data)
        backup=None
        if current.exists():
            history=archive_root()/'maps'/Path(files[0]).parent.name/current.name
            history.mkdir(parents=True,exist_ok=True)
            backup=Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_'),dir=str(history)))
            shutil.move(str(current),str(backup/'map'))
        try:os.replace(str(candidate),str(current))
        except Exception:
            if backup is not None:shutil.move(str(backup/'map'),str(current))
            raise
    return data


class MappingJob:
    def __init__(self,bag,job,replace=False):
        self.bag=os.path.abspath(bag);self.job=Path(job).resolve();self.replace=replace
        self.child=None;self.cancelled=False;self.started=time.monotonic()
        self.state=dict(state='running',stage='inspect',elapsed_s=0,job=str(self.job))

    def update(self,stage,**extra):
        self.state.update(stage=stage,elapsed_s=time.monotonic()-self.started,**extra)
        atomic_json(self.job/'status.json',self.state)

    def run_command(self,command,stage,capture=False):
        self.update(stage)
        if self.cancelled:raise KeyboardInterrupt()
        log=self.job/(stage+'.log')
        env=dict(os.environ,PYTHONUNBUFFERED='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_IP='127.0.0.1')
        env.pop('ROS_HOSTNAME',None)
        with log.open('wb') as output:
            self.child=subprocess.Popen(command,stdout=output,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            stopping=None
            while self.child.poll() is None:
                if (self.job/'cancel').exists() or self.cancelled:
                    self.cancelled=True
                    if stopping is None:
                        stopping=time.monotonic();self.child.terminate();self.update('stopping')
                self.update('stopping' if self.cancelled else stage)
                time.sleep(.25)
            code=self.child.returncode;self.child=None
        if self.cancelled:raise KeyboardInterrupt()
        if code:raise RuntimeError(log.read_text(errors='replace')[-1800:] or 'Mapping process failed')
        if capture:
            for line in reversed(log.read_text().splitlines()):
                if line.startswith('{'):return json.loads(line)
            raise RuntimeError('Navigation inspection returned no result')

    def run(self):
        import fcntl
        files=session_files(self.bag)
        if not files:raise ValueError('No source bags')
        lockpath=self.job.parent/'mapping.lock'
        with lockpath.open('w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if list(Path(files[0]).parent.glob('*.bag.active')):raise ValueError('Recording is still active')
            if map_artifacts(self.bag) and not self.replace:raise ValueError('A map already exists; replacement was not confirmed')
            pointer=Path(map_manifest_path(files[0]));previous=pointer.read_bytes() if pointer.exists() else None
            sources=source_snapshot(files)
            (self.job/'request.json').write_text(json.dumps(dict(bag=self.bag,sources=sources,replace=self.replace),indent=2))
            probe=self.run_command(['python2',str(HERE/'mapping_probe.py'),self.bag],'inspect',True)
            atomic_json(self.job/'inspection.json',probe)
            self.update('rebuild',partial=probe['partial'],record_bounds=probe['record_bounds'],selected_duration_s=probe['selected']['record_end']-probe['selected']['record_start'])
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
            command=[sys.executable,str(HERE/'rebuild_map.py'),self.bag,'--recompute','--output-dir',str(self.job/'result'),'--port',str(port),'--rate','2']
            for value,flag in zip(probe['record_bounds'],['--record-start','--record-end']):
                if value is not None:command.extend([flag,str(value)])
            self.run_command(command,'rebuild')
            if (self.job/'cancel').exists():raise KeyboardInterrupt()
            self.update('verify');data=publish(self.bag,self.job,sources,previous)
            self.update('complete',state='complete',association=str(pointer),map=str(pointer.parent/data['yaml']))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('bag');ap.add_argument('--job');ap.add_argument('--replace',action='store_true');args=ap.parse_args()
    task=MappingJob(args.bag,args.job or new_job(args.bag),args.replace)
    def cancel(_sig,_frame):task.cancelled=True
    signal.signal(signal.SIGTERM,cancel);signal.signal(signal.SIGINT,cancel)
    try:
        task.run()
        if not args.job:print(json.dumps(task.state,indent=2))
        return 0
    except KeyboardInterrupt:task.update('cancelled',state='cancelled');return 130
    except Exception as exc:task.update('failed',state='failed',error=str(exc));return 1


if __name__=='__main__':sys.exit(main())
