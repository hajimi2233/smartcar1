#!/usr/bin/env python
"""Installed navigation draw/restart/clear test; isolated ROS, sends no goals."""
from __future__ import print_function
import os,time,tempfile,shutil,signal,subprocess,json
import rospy,yaml
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
rospy.init_node('line_persistence_test')
folder=tempfile.mkdtemp();node=None;markers=[]
rospy.set_param('/sim/actuator_model',dict(version=1,wheelbase=.62,front_track=.45,wheel_radius=.09,acceleration=.3,braking=.5,steer_rate=2.5))
with open('/project/config/navigation.yaml') as f:params=yaml.safe_load(f)
params['low_cost_lines_file']=folder+'/lines.json';rospy.set_param('/single_goal_nav',params)
maps=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
points=rospy.Publisher('/single_nav/line_point',PointStamped,queue_size=1)
rospy.Subscriber('/single_nav/zero_cost_line',Marker,lambda m:markers.append(m))
m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=100;m.info.resolution=.05
m.info.origin.orientation.w=1.;m.data=[0]*10000

def wait(check,label):
 start=time.time()
 while time.time()-start<8:
  if check():print('PASS '+label);return
  time.sleep(.05)
 raise AssertionError(label)
def start():
 global node
 markers[:]=[]
 node=subprocess.Popen(['rosrun','smartcar_navigation','single_goal_nav.py'])
 wait(lambda:points.get_num_connections()>0,'draw tool subscriber ready')
 wait(lambda:len(markers)>0,'map restores line markers')
def stop():
 global node
 node.send_signal(signal.SIGINT);node.wait();node=None
 wait(lambda:points.get_num_connections()==0,'navigation stopped')
def draw(x,y):
 p=PointStamped();p.header.frame_id='map';p.point.x=x;p.point.y=y;points.publish(p);time.sleep(.15)
try:
 maps.publish(m);start();draw(1,1);draw(2,1)
 wait(lambda:os.path.exists(folder+'/lines.json') and len(markers[-1].points)==2,'line auto saved and visible')
 stop();rospy.delete_param('/single_nav/zero_cost_lines');start()
 wait(lambda:len(markers[-1].points)==2,'disk restore without ROS line parameters')
 m.data[0]=100;maps.publish(m)
 wait(lambda:markers[-1].action==Marker.DELETE,'different map excludes saved lines')
 m.data[0]=0;maps.publish(m)
 wait(lambda:len(markers[-1].points)==2 and markers[-1].action==Marker.ADD,'return to original map restores lines')
 draw(1,2);draw(2,2)
 wait(lambda:len(markers[-1].points)==4,'draw tool appends after restore')
 response=rospy.ServiceProxy('/single_nav/clear_zero_cost_line',Trigger)();assert response.success,response.message
 wait(lambda:markers[-1].action==Marker.DELETE,'clear removes display')
 stop();start()
 wait(lambda:markers[-1].action==Marker.DELETE,'clear persists across restart')
 print('ALL INSTALLED LOW COST LINE TESTS PASSED')
finally:
 if node:stop()
 rospy.signal_shutdown('done');shutil.rmtree(folder)
