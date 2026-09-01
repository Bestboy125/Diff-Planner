cmake_minimum_required(VERSION 3.0.2)
project(vla_diff_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cv_bridge
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs quadrotor_msgs rospy sensor_msgs std_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/vla_diff_bridge_node.py
  scripts/onboard_observation_uplink_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_vla_fastlio_diff_preview.sh
  scripts/verify_onboard_no_motion.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
