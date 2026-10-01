#!/usr/bin/env python
"""ROS integration test; run with roscore in an isolated container, not a live car."""
from __future__ import print_function
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,ROOT+'/src/smartcar_localization/scripts')
sys.path.insert(0,ROOT+'/src/smartcar_navigation/scripts')
from wall_editor import WallEditor
from wall_localizer import WallLocalizer
from region_editor import Editor
from corridor_monitor import CorridorMonitor


def run():
    rospy.init_node('wall_integration')
    folder=tempfile.mkdtemp()
    rospy.set_param('~file',folder+'/walls.json');editor=WallEditor()
    rospy.set_param('~file',folder+'/region.json');region=Editor()
    localizer=WallLocalizer();monitor=CorridorMonitor()
    status=[{}];corridor=[None]
    rospy.Subscriber('/wall_localization/status',String,lambda m: status.__setitem__(0,json.loads(m.data)))
    rospy.Subscriber('/corridor/state',String,lambda m: corridor.__setitem__(0,m.data))
    maps=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
    scans=rospy.Publisher('/scan',LaserScan,queue_size=1)
    # The node resolves relative scan under the root namespace.
    initial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
    broadcaster=tf.TransformBroadcaster()
    timer=rospy.Timer(rospy.Duration(.02),lambda e: broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),'base_footprint','odom'))
    def scan():
        m=LaserScan();m.header.frame_id='base_footprint';m.header.stamp=rospy.Time.now()  # exercise TF arriving shortly after the scan
        m.angle_min=-math.pi;m.angle_increment=math.pi/360;m.range_min=.05;m.range_max=12.
        ranges=[]
        for i in range(720):
            angle=m.angle_min+i*m.angle_increment;c,s=math.cos(angle),math.sin(angle)
            candidates=[]
            if c>1e-5 and -4<=2*s/c<=4: candidates.append(2/c)
            if s>1e-5 and -4<=2*c/s<=4: candidates.append(2/s)
            ranges.append(min(candidates) if candidates else float('inf'))
        m.ranges=ranges;scans.publish(m)
    def wait(predicate,label,timeout=5.,publish=True):
        start=time.time()
        while time.time()-start<timeout:
            if publish: scan()
            if predicate(): print('PASS '+label);return
            time.sleep(.05)
        raise AssertionError(label+': '+str(status[0])+' corridor='+str(corridor[0]))
    def draw_region(xmax):
        assert region.action('begin').success
        for x,y in ((-1,-1),(xmax,-1),(xmax,1.5),(-1,1.5)):
            p=PointStamped();p.header.frame_id='map';p.point.x=x;p.point.y=y;region.click(p)
        assert region.action('save').success
    def command(*args):
        subprocess.check_call([sys.executable,ROOT+'/src/smartcar_localization/scripts/wall_command.py']+list(args))
    try:
        time.sleep(.5)
        m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=200;m.info.resolution=.05;m.info.origin.orientation.w=1
        m.info.origin.position.x=m.info.origin.position.y=-5.
        grid=np.zeros((200,200),dtype=int)
        grid[140,:]=100;grid[:,140]=100
        m.data=grid.ravel().tolist()
        maps.publish(m);wait(lambda: editor.map_id is not None and localizer.map_id is not None,'map received',publish=False)
        draw_region(1.5)
        for group,offset in (('inside',0),('outside',.15)):
            command(group)
            for x,y in ((-4,2+offset),(4,2+offset),(2+offset,-4),(2+offset,4)):
                p=PointStamped();p.header.frame_id='map';p.point.x=x;p.point.y=y;editor.click(p)
        command('preview');command('save');command('list')
        # The target is the map cell boundary surface, not the drawn y=2 line.
        assert any(abs(p[1]-2.025)<.01 for p in editor.active['inside'][0]['samples'])
        with open(folder+'/walls.json') as stream: assert json.load(stream)['schema_version']==3
        assert os.path.exists(folder+'/walls.json')
        import copy
        active=copy.deepcopy(editor.active)
        with open(folder+'/walls.json') as stream: persisted=stream.read()
        editor.draft['inside'].append(dict(id=99,start=[-4,-4],end=[-3,-4]))
        try: editor.execute(['save']);raise AssertionError('Empty-map selection accepted')
        except ValueError: pass
        assert editor.active==active
        with open(folder+'/walls.json') as stream: assert stream.read()==persisted
        editor.execute(['cancel'])
        # Old files store hand lines; load must re-extract instead of trusting them.
        legacy=dict(schema_version=1,frame_id='map',map_sha256=editor.map_id,groups=editor.saved_selectors)
        with open(folder+'/walls.json','w') as stream: json.dump(legacy,stream)
        editor.execute(['load'])
        assert any(abs(p[1]-2.025)<.01 for p in editor.active['inside'][0]['samples'])
        editor.execute(['save'])
        print('PASS failed save is atomic; legacy selectors re-extracted')
        command('delete','inside','2');assert len(editor.draft['inside'])==1
        assert len(editor.active['inside'])==2
        command('cancel');assert len(editor.draft['inside'])==2
        command('load')
        p=PoseWithCovarianceStamped();p.header.frame_id='map';p.pose.pose.position.x=.1;p.pose.pose.position.y=-.1
        p.pose.pose.orientation.w=1.;initial.publish(p)
        wait(lambda: status[0].get('state')=='MATCHED' and status[0].get('group')=='inside','inside feature matching')
        pose=localizer.read_tf('map','base_footprint',rospy.Time(0))
        np.testing.assert_allclose(pose,(.025,.025,0),atol=.01)
        wait(lambda: corridor[0]=='INSIDE','corridor consumes feature TF')
        draw_region(-.2)
        wait(lambda: status[0].get('state')=='MATCHED' and status[0].get('group')=='outside','outside feature switching')
        pose=localizer.read_tf('map','base_footprint',rospy.Time(0))
        np.testing.assert_allclose(pose,(.025,.025,0),atol=.01)
        command('clear','outside');command('save')
        wait(lambda: status[0].get('state')=='ODOM_ONLY','explicit coast when features removed')
        wait(lambda: status[0].get('state')=='LOST','feature loss stops map TF',timeout=4.)
        wait(lambda: corridor[0]=='UNKNOWN','stale localization is not outside')
        m.data[0]=100;maps.publish(m)
        wait(lambda: localizer.correction is None and not localizer.groups['inside'],'map change invalidates feature data')
        print('ALL ROS WALL TESTS PASSED')
    finally:
        timer.shutdown();rospy.signal_shutdown('finished');shutil.rmtree(folder)


if __name__=='__main__': run()
