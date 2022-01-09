cmake_minimum_required(VERSION 2.8.3)
project(panda_to_yolo)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp rospy)

add_service_files(FILES pandaToYolo.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES panda_to_yolo
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})  
