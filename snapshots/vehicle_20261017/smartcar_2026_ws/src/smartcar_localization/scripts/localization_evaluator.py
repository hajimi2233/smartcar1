#!/usr/bin/env python
"""Read-only truth comparison; never publishes TF, pose, initialpose or commands."""
import bisect
import json
import math
import os
import threading
from collections import deque
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from wall_features import compose, inverse
from wheel_distance import wrap


class Evaluator(object):
    def __init__(self):
        self.lock=threading.RLock();self.truth=deque();self.pending={};self.first=None
        self.sums={k:dict(count=0,position_sq=0.,yaw_sq=0.,relative_sq=0.,max_position=0.,delay_sum=0.) for k in ('lidar','fused')}
        self.transform=rospy.get_param('~truth_to_map',[0.,0.,0.])
        if len(self.transform)!=3: raise ValueError('truth_to_map must be [x,y,yaw]')
        self.path=rospy.get_param('~output')
        directory=os.path.dirname(os.path.abspath(self.path))
        if not os.path.isdir(directory): os.makedirs(directory)
        # Never overwrite a previous experiment.
        self.stream=open(self.path,'a');self.closed=False;self.run=rospy.Time.now().to_sec()
        self.publisher=rospy.Publisher('/localization_eval/summary',String,queue_size=1,latch=True)
        rospy.Subscriber('/sim/ground_truth/odom',Odometry,self.on_truth,queue_size=200)
        rospy.Subscriber('/wall_localization/pose',PoseWithCovarianceStamped,lambda m:self.on_pose('fused',m),queue_size=20)
        rospy.Subscriber('/lidar_baseline/pose',PoseWithCovarianceStamped,lambda m:self.on_pose('lidar',m),queue_size=20)
        rospy.on_shutdown(self.close)

    def close(self):
        with self.lock:
            self.closed=True
            self.stream.close()

    @staticmethod
    def pose(p):
        q=p.orientation
        return (p.position.x,p.position.y,tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2])

    def on_truth(self,msg):
        with self.lock:
            stamp=msg.header.stamp.to_sec()
            if self.truth and stamp<=self.truth[-1][0]:
                self.truth.clear();self.pending.clear();self.first=None
                for values in self.sums.values():
                    for key in values: values[key]=0
            self.truth.append((stamp,compose(self.transform,self.pose(msg.pose.pose))))
            while self.truth and stamp-self.truth[0][0]>20: self.truth.popleft()
            self.flush()

    def on_pose(self,kind,msg):
        with self.lock:
            stamp=msg.header.stamp.to_sec()
            self.pending.setdefault(stamp,{})[kind]=(self.pose(msg.pose.pose),(rospy.Time.now()-msg.header.stamp).to_sec())
            self.flush()

    def flush(self):
        if self.closed or len(self.truth)<2: return
        data=list(self.truth);times=[x[0] for x in data]
        for stamp in sorted(list(self.pending)):
            estimates=self.pending[stamp]
            if stamp<times[0]: del self.pending[stamp];continue
            if len(estimates)!=2:
                if times[-1]-stamp>5: del self.pending[stamp]
                continue
            i=bisect.bisect_left(times,stamp)
            if i==len(times): continue
            if times[i]==stamp: truth=data[i][1]
            else:
                a,b=data[i-1],data[i];fraction=(stamp-a[0])/(b[0]-a[0])
                if b[0]-a[0]>.2: del self.pending[stamp];continue
                truth=(a[1][0]+fraction*(b[1][0]-a[1][0]),a[1][1]+fraction*(b[1][1]-a[1][1]),a[1][2]+fraction*wrap(b[1][2]-a[1][2]))
            if self.first is None: self.first=(truth,{k:v[0] for k,v in estimates.items()})
            row=dict(run=self.run,stamp=stamp,truth=truth,truth_to_map=self.transform)
            relative_truth=compose(inverse(self.first[0]),truth)
            for kind,(pose,delay) in estimates.items():
                error=math.hypot(pose[0]-truth[0],pose[1]-truth[1]);yaw=wrap(pose[2]-truth[2])
                relative=compose(inverse(self.first[1][kind]),pose)
                rel_error=math.hypot(relative[0]-relative_truth[0],relative[1]-relative_truth[1])
                row[kind]=dict(pose=pose,position_error=error,yaw_error=yaw,relative_error=rel_error,delay=delay)
                acc=self.sums[kind];acc['count']+=1;acc['position_sq']+=error**2;acc['yaw_sq']+=yaw**2
                acc['relative_sq']+=rel_error**2;acc['delay_sum']+=delay;acc['max_position']=max(acc['max_position'],error)
            self.stream.write(json.dumps(row)+'\n');self.stream.flush()
            result={}
            for kind,a in self.sums.items():
                n=a['count'];result[kind]=dict(samples=n,position_rmse_m=math.sqrt(a['position_sq']/n),
                    relative_rmse_m=math.sqrt(a['relative_sq']/n),yaw_rmse_deg=math.degrees(math.sqrt(a['yaw_sq']/n)),
                    max_position_m=a['max_position'],mean_output_age_s=a['delay_sum']/n)
            self.publisher.publish(json.dumps(result,sort_keys=True));del self.pending[stamp]


if __name__=='__main__':
    rospy.init_node('localization_evaluator');Evaluator();rospy.spin()
