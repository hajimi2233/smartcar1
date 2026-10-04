cmake_minimum_required(VERSION 2.8.3)
project(smartcar_navigation)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/single_goal_nav.py
  scripts/corridor_monitor.py
  scripts/region_editor.py
  scripts/send_single_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
configure_file(scripts/ackermann_core.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ackermann_core.py COPYONLY)
configure_file(scripts/local_planner.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/local_planner.py COPYONLY)
configure_file(scripts/path_tracking.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/path_tracking.py COPYONLY)
configure_file(scripts/actuator_model.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/actuator_model.py COPYONLY)
configure_file(scripts/ground_truth_pose.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ground_truth_pose.py COPYONLY)
configure_file(scripts/nav_config.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nav_config.py COPYONLY)
configure_file(scripts/nav_obstacles.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nav_obstacles.py COPYONLY)
configure_file(scripts/region_geometry.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/region_geometry.py COPYONLY)
install(FILES scripts/ackermann_core.py scripts/local_planner.py scripts/path_tracking.py scripts/actuator_model.py scripts/ground_truth_pose.py scripts/nav_config.py scripts/nav_obstacles.py scripts/region_geometry.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

configure_file(scripts/corridor_geometry.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/corridor_geometry.py COPYONLY)
install(FILES scripts/corridor_geometry.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

configure_file(scripts/low_cost_lines.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/low_cost_lines.py COPYONLY)
install(FILES scripts/low_cost_lines.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
