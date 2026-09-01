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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
