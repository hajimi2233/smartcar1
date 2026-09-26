#!/usr/bin/env python
from __future__ import print_function
import math, time, json
import rospy, tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from std_msgs.msg import String

rospy.init_node('verify_inspection_sim',anonymous=True)
pub=rospy.Publisher('/sim/cmd_vel',Twist,queue_size=1)
rospy.wait_for_service('/gazebo/get_model_state',timeout=20)
model=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
world=rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
listener=tf.TransformListener()
samples=[]; joint_samples=[]
sub=rospy.Subscriber('/sim/scan',LaserScan,lambda msg:samples.append(msg))
js_sub=rospy.Subscriber('/sim/joint_states',JointState,lambda msg:joint_samples.append(msg))
def state():
    m=model('inspection_car','world');assert m.success
    return m
def yaw(s):
    q=s.pose.orientation
    return tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
def drive(v,w,duration):
    t=time.time();m=Twist();m.linear.x=v;m.angular.z=w
    while time.time()-t<duration and not rospy.is_shutdown():pub.publish(m);time.sleep(.04)
def local_distance(a,b):
    return (b.pose.position.x-a.pose.position.x)*math.cos(yaw(a))+(b.pose.position.y-a.pose.position.y)*math.sin(yaw(a))
report={}
try:
    time.sleep(2)
    status=rospy.wait_for_message('/inspection/preview_status',String,timeout=10).data
    assert status.startswith('PLAN ONLY'),status
    assert len(samples)>3,'No laser data'
    scan=samples[-1]
    assert scan.header.frame_id=='laser',scan.header.frame_id
    assert len(scan.ranges)==360
    valid=[r for r in scan.ranges if not math.isinf(r) and not math.isnan(r) and scan.range_min<=r<=scan.range_max]
    assert len(valid)>100,'Lidar not seeing walls'
    listener.waitForTransform('sim_world','laser',rospy.Time(0),rospy.Duration(5))
    listener.waitForTransform('sim_world','front_axle_midpoint',rospy.Time(0),rospy.Duration(5))
    report['laser_valid_returns']=len(valid)
    report['laser_hz_sim_time']=(len(samples)-1)/(samples[-1].header.stamp-samples[0].header.stamp).to_sec()
    a=state();drive(.15,0,2);drive(0,0,1);b=state()
    report['forward_m']=local_distance(a,b);assert report['forward_m']>.15
    a=b;drive(-.15,0,2);drive(0,0,1);b=state()
    report['reverse_m']=local_distance(a,b);assert report['reverse_m']<-.15
    a=b;drive(0,.4,1);drive(0,0,.3);b=state()
    report['stationary_yaw_change']=abs(yaw(b)-yaw(a));assert report['stationary_yaw_change']<.03
    a=b;drive(.12,.1,2);drive(0,0,1);b=state()
    report['turn_yaw_change']=math.atan2(math.sin(yaw(b)-yaw(a)),math.cos(yaw(b)-yaw(a)))
    assert report['turn_yaw_change']>.03
    maximum=max(abs(pos) for js in joint_samples for name,pos in zip(js.name,js.position) if 'steer' in name)
    report['max_front_wheel_angle_deg']=math.degrees(maximum);assert maximum<=math.radians(40)+.002
    drive(.12,0,1)
    # Stop sending: verify the wall-clock watchdog stops drive without continued commands.
    time.sleep(2)
    stopped=state();report['watchdog_speed']=math.hypot(stopped.twist.linear.x,stopped.twist.linear.y)
    assert report['watchdog_speed']<.03
    t=time.time();sim_start=world().sim_time;time.sleep(5)
    report['real_time_factor']=(world().sim_time-sim_start)/(time.time()-t)
    report['passed']=True
finally:
    pub.publish(Twist())
    print(json.dumps(report,indent=2,sort_keys=True))
