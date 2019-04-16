cmake_minimum_required(VERSION 2.8.3)
project(groceries)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  move_base_msgs
  actionlib
  actionlib_msgs
  hsr_manipulation_2019
)

catkin_python_setup()

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   std_srvs
#   actionlib_msgs
#)

catkin_package(
 #       INCLUDE_DIRS include
 #       LIBRARIES libvilla_hsrb
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  hsr_kinematics
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
  actionlib_msgs
  hsr_manipulation_2019
)

#include_directories(
#    ${catkin_INCLUDE_DIRS}
#        include
#)

#install(DIRECTORY scripts/
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)

catkin_install_python(PROGRAMS
    scripts/storing_groceries_2019
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

