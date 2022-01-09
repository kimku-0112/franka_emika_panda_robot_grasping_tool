cmake_minimum_required(VERSION 2.8.3)
project(panda_planing)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp rospy 
geometry_msgs) 

add_service_files(FILES 
goal_joint.srv
goal_pose.srv
grasp.srv
pandaToYolo.srv
)
generate_messages(DEPENDENCIES 
geometry_msgs
std_msgs
)

catkin_package(
  LIBRARIES panda_planing
  CATKIN_DEPENDS std_msgs geometry_msgs roscpp rospy
)

include_directories(${catkin_INCLUDE_DIRS})  

add_executable(panda_planing src/panda_planing_service.cpp)
add_dependencies(panda_planing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(panda_planing ${catkin_LIBRARIES})
