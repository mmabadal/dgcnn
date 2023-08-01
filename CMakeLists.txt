cmake_minimum_required(VERSION 2.8.3)
project(dgcnn)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY 
  msg
  FILES
  info_bb.msg
  info_bbs.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


