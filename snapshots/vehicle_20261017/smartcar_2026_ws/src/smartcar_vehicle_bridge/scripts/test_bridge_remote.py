from __future__ import print_function
import sys,imp,threading,json
sys.path.insert(0,'/home/smartcar/smartcar_2026_ws/src/smartcar_vehicle_bridge/scripts')
from feedback_monitor import PulseMonitor
import rospy
mod=imp.load_source('bridge_contract',sys.path[0]+'/vehicle_bridge.py')
now=[10.];mod.clock=lambda:now[0]
class Pub(object):
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)
def make():
    b=mod.Bridge.__new__(mod.Bridge);b.lock=threading.RLock()
    b.K=93313.;b.max_v=.5;b.middle=2110.;b.scale=652.
    b.L=.62;b.T=.445;b.Tr=.445;b.radius=.09
    b.steering=[[1457.,-.8879199224],[2763.,.8370926754]]
    b.steer=1762.;b.steer_time=10.;b.tick_time=10.;b.velocity=.2
    b.velocity_stamp=rospy.Time(100);b.velocity_base_frame='base_footprint'
    b.last_cmd=10.;b.target=(.1,.53);b.fresh=.3;b.timeout=.4;b.owner_ok=True;b.owner_reason='';b.enabled=True
    b.hold_on_fault=True;b.fault_steering=None;b.last_sent_steering=None
    b.pulse_monitor=PulseMonitor(b.K,b.max_v);b.total=0.
    b.preview=Pub();b.out=Pub();b.status=Pub();b.feedback=Pub()
    b.update_velocity=lambda t:None
    return b
b=make()
for t,count in [(10.,0),(10.09,0),(12.97,59256)]:
    now[0]=t;b.last_cmd=t;b.steer_time=t
    b.on_ticks(type('M',(),{'data':count})());b.on_timer(None)
    assert b.velocity==.2 and b.out.messages[-1].linear.x==.1
    assert json.loads(b.status.messages[-1].data)['state']=='READY'
assert b.pulse_monitor.state=='DELAYED_UPDATE'
assert abs(b.pulse_monitor.average-.214)<.002
assert json.loads(b.status.messages[-1].data)['wheel_velocity_estimated'] is True
for fault in ('lidar','serial','command','ownership'):
    now[0]=10.;b=make()
    if fault=='lidar':b.velocity=None
    elif fault=='serial':b.tick_time=9.
    elif fault=='command':b.last_cmd=9.
    else:b.owner_ok=False
    b.on_timer(None)
    if fault=='ownership':assert not b.out.messages
    else:
        m=b.out.messages[-1]
        assert m.linear.x==0. and abs(m.angular.z-(2110.-1762.)/652.)<1e-10
print('PASS: accumulated pulses do not interrupt lidar feedback; stale pose, serial, command and ownership protections remain')
