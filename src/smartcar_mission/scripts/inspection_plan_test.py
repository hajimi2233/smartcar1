#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interactive C task plan + existing planning-only navigation test."""
from __future__ import print_function, unicode_literals
import json
import os
import signal
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import rospy
import rosnode
from std_msgs.msg import String
from std_srvs.srv import Trigger
from plan_io import load_plan

try:
    read_input = raw_input
    text_type = unicode
except NameError:
    read_input = input
    text_type = str


def ask(prompt):
    print(prompt, end='')
    sys.stdout.flush()
    return read_input()


def stop_launch(child):
    if child is None or child.poll() is not None:
        return
    os.killpg(child.pid, signal.SIGINT)
    deadline = time.time()+15
    while child.poll() is None and time.time()<deadline:
        time.sleep(.1)
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGTERM)
    child.wait()


def run_session(session_dir):
    args = rospy.myargv()
    if len(args) != 3:
        sys.exit('Usage: inspection_plan_test.py /absolute/points.csv /absolute/navigation.yaml')
    source, config = args[1:]
    if not os.path.isfile(source) or not os.path.isfile(config):
        sys.exit('Missing points.csv or navigation configuration. Save your 16 points first.')
    rospy.init_node('inspection_plan_test', disable_signals=True)
    nodes = rosnode.get_node_names()
    if any(n in nodes for n in ('/single_goal_nav','/multi_goal_nav','/inspection_sim_keyboard')):
        sys.exit('Stop nav, multi and keyboard terminals first; keep start/localization and RViz running.')
    if '/gazebo' not in nodes or '/wall_localizer' not in nodes:
        sys.exit('Run sim.sh start and initialize localization first.')
    print('编号固定从右到左：1/2 在最右列，9/10 在最左列。')
    print('仅测试全局规划：沿用 nav-test 虚拟到达，不驱动车辆。起始位姿取当前仿真定位。')
    print('本次程序内复用低代价线；退出后不保留。需要重画时运行 sim.sh lines-clear。')
    while not rospy.is_shutdown():
        layout = ask('输入 A/B 配置（例如 A1A2B3B4A5B6A7A8B9B10；q 退出）：').strip()
        if layout.lower() == 'q': break
        if not layout: continue
        try:
            raw = subprocess.check_output(['rosrun','smartcar_mission','inspection_plan14',layout,source], stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as exc:
            print('配置无效：'+exc.output.decode('utf-8','replace')); continue
        fd, path = tempfile.mkstemp(prefix='inspection-test-',suffix='.jsonl')
        child, sub = None, None
        result, events, done = [], [], threading.Event()
        def status(msg):
            events.append(msg.data)
            print('[任务] '+msg.data)
            if msg.data.startswith(('SUCCEEDED: all','FAILED at goal','CANCELLED:')):
                result.append(msg.data); done.set()
        try:
            with os.fdopen(fd,'wb') as stream: stream.write(raw)
            points, labels = load_plan(path)
            print('任务顺序：'+' -> '.join(labels))
            for label,(x,y,yaw) in zip(labels,points):
                print('%-12s (%.3f, %.3f), %.2f deg' % (label,x,y,yaw*180./3.141592653589793))
            sub = rospy.Subscriber('/multi_nav/status',String,status,queue_size=100)
            child = subprocess.Popen(['roslaunch','smartcar_bringup','planning_test.launch',
                                      'config_file:='+config,'multi:=true','plan_file:='+path,
                                      'low_cost_lines_file:='+os.path.join(session_dir,'lines.json'),
                                      'load_saved_low_cost_lines:=true'],preexec_fn=os.setsid)
            deadline=time.time()+30
            while not any(e.startswith('READY:') for e in events):
                if child.poll() is not None or time.time()>deadline:
                    raise RuntimeError('测试节点启动失败或超时')
                time.sleep(.1)
            rospy.wait_for_service('/multi_goal_nav/execute',timeout=10)
            if not (rospy.get_param('/single_goal_nav/ground_truth_test',False)
                    and rospy.get_param('/single_goal_nav/test_auto_arrive',False)):
                raise RuntimeError('未启用虚拟到达，拒绝执行测试')
            answer = ask('可画低代价线（后续轮复用），不画则使用普通导航；回车开始测试，q 取消本轮：').strip()
            if answer.lower() == 'q': continue
            response=rospy.ServiceProxy('/multi_goal_nav/execute',Trigger)()
            if not response.success: raise RuntimeError(response.message)
            while not done.wait(.2):
                if child.poll() is not None: raise RuntimeError('测试节点意外退出')
            print('本轮结果：'+result[-1])
            report={'layout':layout,'points_file':source,'targets':labels,'result':result[-1],
                    'events':events,'planning_only':True,'time':time.time()}
            report_path=os.path.join(os.path.dirname(source),'last_plan_test.json')
            with open(report_path,'w') as stream: json.dump(report,stream,indent=2)
            print('结果保存到 '+report_path)
            ask('可在 RViz 检查目标箭头（失败目标为红色）；按回车结束本轮：')
        except (ValueError, RuntimeError, rospy.ROSException) as exc:
            print('测试失败：'+text_type(exc))
        finally:
            stop_launch(child)
            if sub is not None: sub.unregister()
            os.unlink(path)


def main():
    session_dir = tempfile.mkdtemp(prefix='plan-test-session-')
    try:
        run_session(session_dir)
    finally:
        shutil.rmtree(session_dir)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n已退出计划测试，仿真和定位继续运行。')
