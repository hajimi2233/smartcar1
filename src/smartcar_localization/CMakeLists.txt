cmake_minimum_required(VERSION 2.8.3)
project(smartcar_localization)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/wall_editor.py scripts/wall_localizer.py scripts/wall_command.py scripts/sim_wheel_pulses.py scripts/localization_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
configure_file(scripts/wall_features.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/wall_features.py COPYONLY)
install(FILES scripts/wall_features.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

configure_file(scripts/map_wall_extraction.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/map_wall_extraction.py COPYONLY)
install(FILES scripts/map_wall_extraction.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

configure_file(scripts/wheel_distance.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/wheel_distance.py COPYONLY)
install(FILES scripts/wheel_distance.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
