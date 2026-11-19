#!/usr/bin/env python3
"""Private ROS mapping, failed/cancelled replacement and reader-coordinate integration."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

root=Path(sys.argv[1]).resolve();sys.path.insert(0,str(root))
from mapping_job import new_job,read_status
from viewer_core import map_manifest_path
from ros_runtime import ros_python

fixture='''import rosbag,rospy,math,sys
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from tf2_msgs.msg import TFMessage
with rosbag.Bag(sys.argv[1],'w') as b:
 for parent,child in [('base_footprint','base_link'),('base_link','laser')]:
  tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child;tr.transform.rotation.w=1.
  b.write('/tf_static',TFMessage([tr]),rospy.Time.from_sec(1000.))
 old=OccupancyGrid();old.header.frame_id='map';old.info.width=old.info.height=2;old.info.resolution=1.;old.info.origin.orientation.w=1.;old.data=[0,100,-1,-1]
 b.write('/map',old,rospy.Time.from_sec(1000.))
 for i in range(100):
  stamp=rospy.Time.from_sec(1000.1+i*.1);x=.3*math.sin(i*.06);y=.15*math.cos(i*.06)
  scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi;scan.angle_increment=math.pi*2/360;scan.angle_max=math.pi-scan.angle_increment;scan.range_min=.1;scan.range_max=10.
  ranges=[]
  for j in range(360):
   angle=scan.angle_min+j*scan.angle_increment;c=math.cos(angle);s=math.sin(angle);hits=[]
   if abs(c)>1e-8:hits.append(((4-x)/c) if c>0 else ((-4-x)/c))
   if abs(s)>1e-8:hits.append(((3-y)/s) if s>0 else ((-3-y)/s))
   ranges.append(min(v for v in hits if v>0))
  scan.ranges=ranges;b.write('/scan',scan,stamp)
  odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.w=1.;b.write('/odom',odom,stamp)
  trs=[]
  for parent,child,tx,ty in [('odom','base_footprint',x,y),('map','odom',1000.,1000.)]:
   tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child;tr.header.stamp=stamp;tr.transform.translation.x=tx;tr.transform.translation.y=ty;tr.transform.rotation.w=1.;trs.append(tr)
  b.write('/tf',TFMessage(trs),stamp)
'''


def wait_worker(process,job,timeout=120):
    try:code=process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        (job/'cancel').touch();process.wait(timeout=120);raise
    return code,read_status(job)


def spawn(bag,job):
    return subprocess.Popen([sys.executable,str(root/'mapping_job.py'),str(bag),'--job',str(job),'--replace'],
                            stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)


def read_frame(bag):
    p=subprocess.Popen([ros_python(),str(root/'bag_viewer_server.py'),str(bag)],stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
    try:
        stdout,stderr=p.communicate('{"t":3,"frame":"map"}\n{"cmd":"close"}\n',timeout=20)
        rows=[json.loads(s) for s in stdout.splitlines() if s.startswith('{')]
        assert len(rows)>=2,(stdout,stderr)
        return rows[0],rows[1]
    finally:
        if p.poll() is None:p.kill();p.wait()


with tempfile.TemporaryDirectory(prefix='mapping-ros-integration-') as temp:
    folder=Path(temp);bag=folder/'cam_20261115_120000_0.bag'
    os.environ['BAGTOOLS_ARCHIVE_DIR']=str(folder/'archive')
    generator=folder/'fixture.py';generator.write_text(fixture)
    subprocess.run([ros_python(),str(generator),str(bag)],check=True)
    original=hashlib.sha256(bag.read_bytes()).hexdigest()
    pointer=Path(map_manifest_path(str(bag)));pointer.parent.mkdir(parents=True)
    old=b'{"legacy_static_map":true}';pointer.write_bytes(old)
    job=new_job(str(bag));code,status=wait_worker(spawn(bag,job),job)
    assert code==0 and status['state']=='complete',status
    assert (job/'previous_association.json').read_bytes()==old
    current=pointer.read_bytes();linked=json.loads(current)
    assert linked['map_origin']=='recomputed_from_session' and linked['replace_map_odom']
    info,frame=read_frame(bag)
    assert info['world']=='map' and info['map_info'] and frame['pose'],(info,frame)
    assert abs(frame['pose'][0])<10 and abs(frame['pose'][1])<10,frame  # Old AMCL was at (1000,1000).
    linked['record_bounds']=[1002.,1005.];pointer.write_text(json.dumps(linked))
    info,frame=read_frame(bag)
    assert info['map_note'] and abs(info['dur']-3.)<1e-6 and info['map_info'],info
    linked['record_bounds']=[2000.,2010.];pointer.write_text(json.dumps(linked))
    # The reader must reject requests to use the old map outside the rebuilt interval.
    p=subprocess.Popen([ros_python(),str(root/'bag_viewer_server.py'),str(bag)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
    stdout,stderr=p.communicate('{"cmd":"close"}\n',timeout=20)
    info=json.loads(stdout.splitlines()[0]);assert info['world']=='odom' and info['map_info'] is None and info['warning'],info
    pointer.write_bytes(current)
    # Cancel after the private mapper is running, exercising nested cleanup.
    cancelled=new_job(str(bag));process=spawn(bag,cancelled);deadline=time.monotonic()+45
    while not list((cancelled/'result/navigation').glob('*.mapping.bag.active')):
        if process.poll() is not None:raise AssertionError(read_status(cancelled))
        if time.monotonic()>deadline:raise AssertionError('Private mapper did not start')
        time.sleep(.1)
    (cancelled/'cancel').touch();code,status=wait_worker(process,cancelled)
    assert status['state']=='cancelled' and pointer.read_bytes()==current,status
    # An invalid navigation bag must fail without touching an existing pointer.
    invalid=folder/'invalid';invalid.mkdir();bad=invalid/'cam_20261115_130000_0.bag'
    subprocess.run([ros_python(),'-c',"import rosbag,sys; rosbag.Bag(sys.argv[1],'w').close()",str(bad)],check=True)
    bad_pointer=Path(map_manifest_path(str(bad)));bad_pointer.parent.mkdir(parents=True);bad_pointer.write_bytes(old)
    failed=new_job(str(bad));code,status=wait_worker(spawn(bad,failed),failed)
    assert status['state']=='failed' and bad_pointer.read_bytes()==old,status
    assert hashlib.sha256(bag.read_bytes()).hexdigest()==original
    print(json.dumps(dict(state='passed',checks=['fresh_map_without_camera','confirmed_replacement_backup',
        'old_amcl_not_reused','reader_uses_new_map','reader_clips_partial_interval','outside_interval_degrades',
        'cancel_preserves_active_map','invalid_data_preserves_active_map','source_checksum_unchanged']),indent=2))
