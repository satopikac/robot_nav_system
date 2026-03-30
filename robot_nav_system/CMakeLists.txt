cmake_minimum_required(VERSION 3.0.2)
project(robot_nav_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  message_generation
)

add_message_files(FILES
  SystemState.msg
)

add_service_files(FILES
  SwitchMode.srv
  ExecuteTask.srv
  GetSystemStatus.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib actionlib_msgs geometry_msgs move_base_msgs message_runtime
)

catkin_python_setup()
