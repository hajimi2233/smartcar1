"""Container-side preflight for the host's short simulation navigation command."""
from __future__ import print_function
import os
import sys
import time
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

import rosgraph
import rosnode
import rospy


def fail(message):
    sys.exit(message + '\nStop the conflicting launch terminal with Ctrl+C, then retry sim.sh nav.')


def main():
    stage, map_file, lines_file = sys.argv[1:]
    if stage not in ('start', 'nav'):
        sys.exit('Expected start or nav.')
    if not os.path.isfile(map_file):
        sys.exit('Missing map: ' + map_file)
    master = rosgraph.Master('/smartcar_quick_start')
    deadline = time.time() + 45
    while True:
        try:
            nodes = set(rosnode.get_node_names())
            if '/gazebo' in nodes and rospy.has_param('/sim/actuator_model'):
                break
        except Exception:
            pass
        if time.time() >= deadline:
            sys.exit('Gazebo did not become ready within 45 seconds. Run sim.sh logs.')
        time.sleep(.5)

    conflicts = nodes.intersection(['/slam_gmapping', '/amcl', '/multi_goal_nav'])
    if conflicts:
        fail('Conflicting ROS nodes: ' + ', '.join(sorted(conflicts)))
    localization = '/wall_localizer' in nodes
    if localization != ('/slam_map_server' in nodes):
        fail('An incomplete localization stage is already running.')
    if localization:
        # Verify the actual map-server process, not stale parameters from an old launch.
        uri = rosnode.get_api_uri(master, '/slam_map_server')
        pid = ServerProxy(uri).getPid('/smartcar_quick_start')[2]
        with open('/proc/{}/cmdline'.format(pid), 'rb') as stream:
            args = stream.read().decode('utf-8').split('\0')
        if map_file not in args:
            fail('Existing localization uses a different map.')
        if rospy.get_param('/wall_localizer/target_mode', '') != 'full_map':
            fail('Existing localization does not use the default full-map mode.')
        print('Reusing the running full-map localization.')
    elif '/laser_scan_matcher_node' in nodes:
        fail('A separate laser odometry stage is already running.')
    root = os.environ['SMARTCAR_ROOT']
    if stage == 'start':
        if localization:
            print('Simulation and localization are already running.')
            return
        if '/single_goal_nav' in nodes:
            fail('Navigation is running without localization.')
        sys.stdout.flush()
        os.execvp('roslaunch', ['roslaunch', '/tmp/smartcar-quick-localization.launch',
                              'root:=' + root, 'map_file:=' + map_file])
        return
    if not localization:
        sys.exit('Localization is not running. Run sim.sh start in another terminal first.')
    if '/inspection_sim_keyboard' in nodes:
        fail('Keyboard control is still running. Stop it before navigation.')
    if '/single_goal_nav' in nodes:
        default_lines = '/home/hajimi/smartcar/data/navigation/low_cost_lines.json'
        active_lines = rospy.get_param('/single_goal_nav/low_cost_lines_file', default_lines)
        if not localization or active_lines != lines_file:
            fail('Existing navigation has different localization or low-cost-line settings.')
        print('Localization and navigation are already running. Use RViz to set a goal.')
        return
    sys.stdout.flush()
    os.execvp('roslaunch', ['roslaunch', '/tmp/smartcar-quick-nav.launch',
                          'root:=' + root, 'low_cost_lines_file:=' + lines_file])


if __name__ == '__main__':
    main()
