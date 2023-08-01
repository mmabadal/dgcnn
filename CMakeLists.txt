cmake_minimum_required(VERSION 2.8.3)
project(dgcnn)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_message_files(
    DIRECTORY 
    msg
    FILES
    info_bb.msg
    info_bbs.msg
)