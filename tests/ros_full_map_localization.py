#!/usr/bin/env python
"""Exercise the installed launch/node in an isolated ROS master, no simulator truth."""
from __future__ import print_function
import json,math,os,subprocess,tempfile,time,shutil,signal
import numpy as np
import rospy,tf,rosnode
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def run():
    rospy.init_node('full_map_integration')
    folder=tempfile.mkdtemp();process=None;timer=None
    status=[{}];output=[None]
    rospy.Subscriber('/wall_localization/status',String,lambda m:status.__setitem__(0,json.loads(m.data)))
    rospy.Subscriber('/wall_localization/pose',PoseWithCovarianceStamped,lambda m:output.__setitem__(0,m))
    scans=rospy.Publisher('/scan',LaserScan,queue_size=1)
    initial=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
    broadcaster=tf.TransformBroadcaster();listener=tf.TransformListener()
    try:
        grid=np.full((200,200),254,dtype=np.uint8);grid[140,:]=0;grid[:,140]=0
        with open(folder+'/map.pgm','wb') as f:f.write(b'P5\n200 200\n255\n'+np.flipud(grid).tostring())
        with open(folder+'/map.yaml','w') as f:f.write('image: map.pgm\nresolution: 0.05\norigin: [-5, -5, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        process=subprocess.Popen(['roslaunch','smartcar_localization','localization.launch',
            'map_file:='+folder+'/map.yaml','params:=/home/hajimi/smartcar/config/localization.yaml','scan_odometry:=false'])
        timer=rospy.Timer(rospy.Duration(.02),lambda e:broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),'base_footprint','odom'))
        def scan():
            m=LaserScan();m.header.frame_id='base_footprint';m.header.stamp=rospy.Time.now()
            m.angle_min=-math.pi;m.angle_increment=math.pi/360;m.range_min=.05;m.range_max=12.
            for i in range(720):
                a=m.angle_min+i*m.angle_increment;c,s=math.cos(a),math.sin(a);dist=[]
                if c>1e-5 and -4<=2*s/c<=4:dist.append(2/c)
                if s>1e-5 and -4<=2*c/s<=4:dist.append(2/s)
                m.ranges.append(min(dist) if dist else float('inf'))
            scans.publish(m)
        def wait(check,label,timeout=12.):
            start=time.time()
            while time.time()-start<timeout:
                scan()
                if check():print('PASS '+label);return
                time.sleep(.08)
            raise AssertionError(label+': '+str(status[0]))
        wait(lambda:status[0].get('target_mode')=='full_map','default full-map mode')
        nodes=rosnode.get_node_names()
        assert '/amcl' not in nodes and '/wall_editor' not in nodes and '/corridor_monitor' not in nodes,nodes
        time.sleep(.5)
        p=PoseWithCovarianceStamped();p.header.frame_id='map';p.pose.pose.position.x=.10;p.pose.pose.position.y=-.10;p.pose.pose.orientation.w=1.
        initial.publish(p)
        wait(lambda:status[0].get('state')=='MATCHED' and output[0] is not None,'matches without region or selected walls')
        assert status[0]['wheel_state']=='DISABLED'
        pose=output[0].pose.pose
        np.testing.assert_allclose([pose.position.x,pose.position.y],[.025,.025],atol=.012)
        listener.waitForTransform('map','base_footprint',rospy.Time(0),rospy.Duration(2))
        xyz,q=listener.lookupTransform('map','base_footprint',rospy.Time(0))
        np.testing.assert_allclose(xyz[:2],[.025,.025],atol=.012)
        print('PASS installed node corrects initial offset and publishes map->odom')
        old=output[0].header.stamp
        initial.publish(p)
        wait(lambda:output[0].header.stamp>old and status[0].get('state')=='MATCHED','RViz reinitialization')
        print('ALL FULL MAP ROS TESTS PASSED')
    finally:
        if timer:timer.shutdown()
        if process:process.send_signal(signal.SIGINT);process.wait()
        rospy.signal_shutdown('done');shutil.rmtree(folder)

if __name__=='__main__':run()
