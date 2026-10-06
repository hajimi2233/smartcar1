"""Queue selection must not send or cancel a drive command."""
import importlib.util
import math
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/smartcar_mission/scripts'))


def load_mission():
    stubs={}
    for name, names in {'geometry_msgs.msg':['PoseStamped'], 'std_msgs.msg':['String'],
                        'visualization_msgs.msg':['Marker','MarkerArray'],
                        'std_srvs.srv':['Trigger','TriggerResponse']}.items():
        m=types.ModuleType(name)
        for cls in names: setattr(m,cls,type(cls,(),{}))
        stubs[name]=m
    stubs['rospy']=types.SimpleNamespace(Time=lambda n:n)
    stubs['tf']=types.SimpleNamespace(transformations=types.SimpleNamespace(euler_from_quaternion=lambda q:(0.,0.,0.)))
    file=Path(__file__).resolve().parents[1]/'src/smartcar_mission/scripts/multi_goal_nav.py'
    spec=importlib.util.spec_from_file_location('mission_under_test',file)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,stubs): spec.loader.exec_module(module)
    return module


class MissionTests(unittest.TestCase):
    def node(self):
        node=load_mission().MultiGoalNav.__new__(load_mission().MultiGoalNav)
        node.frame='map';node.active=False;node.points=[];node.labels=[];node.failed=False;node.lock=threading.RLock()
        node.status=lambda text:None;node.publish_marks=lambda:None
        return node

    def test_selection_only_records_point_without_command_publishers(self):
        node=self.node()
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=1,frame_id='map'),
             pose=types.SimpleNamespace(position=types.SimpleNamespace(x=2.,y=3.),
             orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        # No goal_pub/cancel_pub exists; any command attempt would fail this test.
        node.on_goal_click(msg)
        self.assertEqual(node.points,[(2.,3.,0.)])
        node.active=True;node.on_goal_click(msg)
        self.assertEqual(len(node.points),1)

    def test_success_advances_queue_one_point_at_a_time(self):
        node=self.node();node.points=[(1.,1.,0.),(2.,2.,0.)]
        node.active=True;node.index=0;node.seen_active_status=False
        sent=[];node.publish_current=lambda:sent.append(node.index)
        msg=lambda text:types.SimpleNamespace(data=text)
        node.on_nav_status(msg('SUCCEEDED: stale'))
        self.assertEqual(sent,[])
        node.on_nav_status(msg('PLANNING'))
        node.on_nav_status(msg('SUCCEEDED: first'))
        self.assertEqual(sent,[1])
        node.on_nav_status(msg('DRIVING: second'))
        node.on_nav_status(msg('SUCCEEDED: second'))
        self.assertFalse(node.active)
        self.assertEqual(node.points,[])


if __name__=='__main__':unittest.main()
