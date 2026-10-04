#!/usr/bin/env python
"""Multi-policy Gazebo experiment. Explicit isolated environment only.

Ground truth appears only in scalar pulse generation and the evaluator. Policies
receive scan/TF/AMCL/maps/pulses, with no reference to the evaluation buffer.
"""
from __future__ import print_function
import os,sys,time,json,math,signal,subprocess,threading,bisect
from collections import deque
import numpy as np
import rospy,tf,rosbag,yaml
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan,JointState
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState,GetPhysicsProperties,SetPhysicsProperties
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,ROOT+'/src/smartcar_localization/scripts')
from wall_features import scan_points,compose,inverse
from map_wall_extraction import map_geometry,extract_groups
from wheel_distance import PulseBuffer,wrap
from localization_roles import Policies,MODES


class Trial(object):
    def __init__(self,output,case,repeat):
        self.output=output;self.case=case;self.repeat=repeat
        self.lock=threading.RLock();self.pulse_lock=threading.RLock();self.truth_lock=threading.RLock()
        self.listener=tf.TransformListener();self.truth=deque();self.amcl=None;self.pulses=PulseBuffer()
        self.engine=None;self.last_stamp=None;self.eval_pending=[];self.rows=[];self.active=True
        self.scan_count=0;self.input_count=0;self.errors=0;self.phase='setup';self.drop=False;self.partial=case['partial'];self.subs=[]
        self.scanpub=rospy.Publisher('/experiment/scan',LaserScan,queue_size=1)
        self.initialpub=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        self.stream=open(output+'/'+case['name']+'_r'+str(repeat)+'.jsonl','w')
        self.bag=rosbag.Bag(output+'/'+case['name']+'_r'+str(repeat)+'.bag','w',compression='bz2');self.bag_lock=threading.RLock()
        m=rospy.wait_for_message('/geometry_map',OccupancyGrid,timeout=10);m.header.frame_id='map'
        geometry=map_geometry(m);cells,samples,res=geometry
        self.all_walls=[dict(id=0,start=[-6,-4],end=[6,4],radius=20.,cells=cells.tolist(),samples=samples.tolist(),resolution=res)]
        selectors=dict(inside=[],outside=[dict(id=1,start=[-5.8,4],end=[5.8,4]),dict(id=2,start=[6,-3.8],end=[6,3.8]),
            dict(id=3,start=[-5.8,-4],end=[5.8,-4]),dict(id=4,start=[-6,-3.8],end=[-6,3.8])])
        if self.partial:selectors['outside']=selectors['outside'][:1]
        if case.get('straight_only'):
            # Exclude perpendicular corner faces from the dilated selection.
            selectors['outside']=[dict(id=1,start=[-4.5,4],end=[4.5,4])]
        self.walls=extract_groups(m,selectors)['outside']
        self.map_pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True);self.map_pub.publish(m)
        self.record('/map',m)
        self.subs.append(rospy.Subscriber('/sim/ground_truth/odom',Odometry,self.on_truth,queue_size=200))
        self.subs.append(rospy.Subscriber('/wheel/pulses',JointState,self.on_pulse,queue_size=200))
        self.subs.append(rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.on_amcl,queue_size=20))
        self.subs.append(rospy.Subscriber('/scan',LaserScan,self.forward_scan,queue_size=1))
        self.subs.append(rospy.Subscriber('/experiment/scan',LaserScan,self.on_scan,queue_size=1))
        self.evaltimer=rospy.Timer(rospy.Duration(.1),self.evaluate)

    def record(self,topic,msg):
        with self.bag_lock:
            if self.active:self.bag.write(topic,msg,rospy.Time.now())

    def tfpose(self,parent,child,stamp):
        xyz,q=self.listener.lookupTransform(parent,child,stamp)
        return (xyz[0],xyz[1],tf.transformations.euler_from_quaternion(q)[2])

    def on_truth(self,m):
        p=m.pose.pose;q=p.orientation
        with self.truth_lock:
            self.truth.append((m.header.stamp.to_sec(),(p.position.x,p.position.y,tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2])))
            while len(self.truth)>2000:self.truth.popleft()
        self.record('/sim/ground_truth/odom',m)

    def on_pulse(self,m):
        with self.pulse_lock:
            try:self.pulses.add(m.header.stamp.to_sec(),m.position[0],m.header.frame_id)
            except ValueError:pass
        self.record('/wheel/pulses',m)

    def forward_scan(self,m):
        if self.engine is not None:self.input_count+=1
        if self.drop:
            import copy
            m=copy.deepcopy(m);m.ranges=[float('inf')]*len(m.ranges)
        self.scanpub.publish(m);self.record('/experiment/scan',m)

    def on_amcl(self,m):
        try:
            stamp=m.header.stamp;p=m.pose.pose;q=p.orientation
            anchor=(p.position.x,p.position.y,tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2])
            odom=self.tfpose('odom','base_footprint',stamp)
            covariance=m.pose.covariance
            self.amcl=(anchor,odom,stamp.to_sec(),max(covariance[0],covariance[7],covariance[35]))
            self.record('/amcl_pose',m)
        except tf.Exception:pass

    def initialize(self):
        p=PoseWithCovarianceStamped();p.header.frame_id='map'
        dx=.35 if self.case['offset'] else .02;dy=-.2 if self.case['offset'] else -.01
        yaw=math.pi+(.15 if self.case['offset'] else .005)
        p.pose.pose.position.x=3.86+dx;p.pose.pose.position.y=2.8+dy
        p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2)
        p.pose.covariance[0]=p.pose.covariance[7]=.25**2;p.pose.covariance[35]=.15**2
        self.initialpub.publish(p);self.record('/initialpose',p)
        odom=self.tfpose('odom','base_footprint',rospy.Time(0))
        with self.lock:
            self.engine=Policies((3.86+dx,2.8+dy,yaw),odom,rospy.Time.now().to_sec())
            self.last_stamp=None

    def on_scan(self,m):
        if self.engine is None:return
        stamp=m.header.stamp.to_sec();phase=self.phase
        started=time.time()
        with self.lock:
            if not self.active:return
            self.scan_count+=1
            try:
                self.listener.waitForTransform('odom','base_footprint',m.header.stamp,rospy.Duration(.25))
                odom=self.tfpose('odom','base_footprint',m.header.stamp)
                laser=self.tfpose('base_footprint',m.header.frame_id,m.header.stamp)
                points,normals=scan_points(m,laser)
                delta=None
                if self.last_stamp is not None:
                    with self.pulse_lock:
                        try:delta=self.pulses.delta(self.last_stamp,stamp)
                        except ValueError:pass
                poses,details=self.engine.update(stamp,odom,points,normals,self.walls,self.all_walls,delta,self.amcl)
                self.last_stamp=stamp
                row=dict(stamp=stamp,phase=phase,poses=poses,details=details,wheel_available=delta is not None,
                         compute_s=time.time()-started,age_s=(rospy.Time.now()-m.header.stamp).to_sec(),scan_odom=odom)
            except (tf.Exception,ValueError,np.linalg.LinAlgError) as exc:
                # An unavailable frontend is not a missing evaluation sample.
                # Score the last published position while explicitly flagging failure.
                self.errors+=1;row=dict(stamp=stamp,phase=phase,error=str(exc),poses=dict(self.engine.poses),
                    details={mode:dict(valid=False,state='FRONTEND_UNAVAILABLE') for mode in MODES})
            self.eval_pending.append(row)

    def evaluate(self,event=None):
        with self.lock:
            with self.truth_lock:data=list(self.truth)
            if len(data)<2:return
            times=[p[0] for p in data]
            for row in list(self.eval_pending):
                stamp=row['stamp'];i=bisect.bisect_left(times,stamp)
                if i==0 or i==len(times):continue
                a,b=data[i-1],data[i];f=(stamp-a[0])/(b[0]-a[0])
                truth=(a[1][0]+f*(b[1][0]-a[1][0]),a[1][1]+f*(b[1][1]-a[1][1]),a[1][2]+f*wrap(b[1][2]-a[1][2]))
                row['truth']=truth;row['errors']={}
                for mode in MODES:
                    if mode in row['poses']:
                        p=row['poses'][mode]
                        row['errors'][mode]=dict(position=math.hypot(p[0]-truth[0],p[1]-truth[1]),yaw=abs(wrap(p[2]-truth[2])),valid=row['details'][mode]['valid'])
                self.rows.append(row);self.stream.write(json.dumps(row)+'\n');self.stream.flush();self.eval_pending.remove(row)

    def close(self):
        self.evaltimer.shutdown()
        for sub in self.subs:sub.unregister()
        with self.lock:
            self.evaluate();self.active=False;self.stream.close()
        with self.bag_lock:self.bag.close()
        # Include prediction-only estimates in max error; also report valid coverage.
        summary={}
        for mode in MODES:
            pairs=[(r,r['errors'][mode]) for r in self.rows if mode in r['errors']]
            errors=[p['position'] for r,p in pairs]
            moving=[p['position'] for r,p in pairs if r['phase']!='initialization']
            summary[mode]=dict(samples=len(errors),received_scans=self.scan_count,input_scans=self.input_count,valid_samples=sum(p['valid'] for r,p in pairs),
                maximum_m=max(errors) if errors else None,maximum_after_init_m=max(moving) if moving else None,
                rmse_m=math.sqrt(sum(v*v for v in errors)/len(errors)) if errors else None,
                p95_m=float(np.percentile(errors,95)) if errors else None,
                max_yaw_deg=math.degrees(max(p['yaw'] for r,p in pairs)) if pairs else None)
        return dict(case=self.case,repeat=self.repeat,modes=summary,tf_errors=self.errors,unpaired=len(self.eval_pending),input_scans=self.input_count,processed_scans=self.scan_count)


def run():
    if '--isolated-sim' not in sys.argv:raise RuntimeError('Isolated ROS master required')
    rospy.init_node('localization_policy_suite');output=os.environ.get('POLICY_OUTPUT','/experiment')
    if not os.path.isdir(output):os.makedirs(output)
    pub=rospy.Publisher('/sim/cmd_vel',Twist,queue_size=1)
    rospy.wait_for_service('/gazebo/set_model_state',timeout=40)
    # Six solvers share one scan callback. Slow wall-clock physics, not ROS time,
    # so their computation does not drop scans or change simulated vehicle speed.
    rospy.wait_for_service('/gazebo/get_physics_properties',timeout=10)
    physics=rospy.ServiceProxy('/gazebo/get_physics_properties',GetPhysicsProperties)()
    rate=float(os.environ.get('POLICY_REALTIME_FACTOR','.4'))/physics.time_step
    result=rospy.ServiceProxy('/gazebo/set_physics_properties',SetPhysicsProperties)(
        physics.time_step,rate,physics.gravity,physics.ode_config)
    if not result.success:raise RuntimeError(result.status_message)
    cases=[dict(name='normal',bias=0.,partial=False,offset=False,outage=False),
           dict(name='plus5',bias=.05,partial=False,offset=False,outage=False),
           dict(name='minus5',bias=-.05,partial=False,offset=False,outage=False),
           dict(name='parallel_plus5',bias=.05,partial=True,offset=False,outage=False),
           dict(name='initial_offset',bias=.05,partial=True,offset=True,outage=False),
           dict(name='scan_outage',bias=.05,partial=True,offset=False,outage=True),
           dict(name='straight_wall_plus5',bias=.05,partial=True,offset=False,outage=False,straight_only=True),
           dict(name='straight_wall_offset',bias=.05,partial=True,offset=True,outage=False,straight_only=True),
           dict(name='straight_wall_outage',bias=.05,partial=True,offset=False,outage=True,straight_only=True),
           dict(name='turn_change_outage',bias=.05,partial=True,offset=False,outage=True,straight_only=True,change_turn=True)]
    wanted=os.environ.get('POLICY_CASES')
    if wanted:cases=[c for c in cases if c['name'] in wanted.split(',')]
    repeats=int(os.environ.get('POLICY_REPEATS','3'));results=[]
    def stage(seconds,v=0.,steering=0.):
        start=rospy.Time.now().to_sec();end=time.time()+seconds*5+10
        while rospy.Time.now().to_sec()-start<seconds and time.time()<end:
            m=Twist();m.linear.x=v;m.angular.z=v*math.tan(steering)/.62;pub.publish(m);time.sleep(.04)
        pub.publish(Twist())
    with open('/project/config/localization.yaml') as f:amcl_params=yaml.safe_load(f)['amcl']
    repeat_start=int(os.environ.get('POLICY_REPEAT_START','0'))
    for repeat in range(repeat_start,repeat_start+repeats):
        for case in cases:
            nodes=[];logs=[];trial=None
            try:
                stage(.3);state=ModelState();state.model_name='inspection_car';state.reference_frame='world'
                state.pose.position.x=3.86;state.pose.position.y=2.8;state.pose.position.z=.015;state.pose.orientation.z=1.
                response=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(state)
                if not response.success:raise RuntimeError(response.status_message)
                stage(.7)
                for key,val in amcl_params.items():rospy.set_param('/amcl/'+key,val)
                rospy.set_param('/amcl/tf_broadcast',False)
                commands=[['roslaunch','smartcar_localization','frontend.launch','scan_topic:=/experiment/scan'],
                          ['rosrun','amcl','amcl','__name:=amcl','scan:=/experiment/scan'],
                          ['rosrun','smartcar_localization','sim_wheel_pulses.py','_scale_error:='+str(case['bias'])]]
                trial=Trial(output,case,repeat)
                for i,cmd in enumerate(commands):
                    log=open(output+'/'+case['name']+'_r'+str(repeat)+'_'+str(i)+'.log','w');logs.append(log)
                    nodes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
                stage(2.);trial.initialize();trial.phase='initialization';stage(3.)
                print('RUN',repeat,case['name']);sys.stdout.flush()
                for phase,seconds,v,steer in [('stationary',2.,0.,0.),('forward',5.,.18,0.),('reverse',5.,-.18,0.),
                                             ('left_turn',5.,.18,.55),('reverse_turn',5.,-.18,.55),('stop',2.,0.,0.)]:
                    trial.phase=phase
                    if case['outage'] and phase=='left_turn':
                        stage(1.,v,steer);trial.drop=True
                        if case.get('change_turn'):
                            stage(.5,v,steer);steer=-steer;stage(.5,v,steer)
                        else:stage(1.,v,steer)
                        trial.drop=False;stage(seconds-2,v,steer)
                    else:stage(seconds,v,steer)
                stage(.3);results.append(trial.close());trial=None
                with open(output+'/results.json','w') as f:json.dump(results,f,indent=2)
                print('DONE',repeat,case['name']);sys.stdout.flush()
            finally:
                pub.publish(Twist())
                if trial is not None:trial.close()
                for node in reversed(nodes):node.send_signal(signal.SIGINT)
                for node in nodes:node.wait()
                for log in logs:log.close()
    print('POLICY SUITE COMPLETE')

if __name__=='__main__':run()
