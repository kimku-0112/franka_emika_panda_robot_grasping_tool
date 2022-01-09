cmake_minimum_required(VERSION 3.0.2)
project(my_cobot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED 
    COMPONENTS
    geometric_shapes
    interactive_markers
    moveit_core
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    pcl_conversions
    pcl_ros
    pluginlib
    rosbag
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    geometry_msgs
    std_msgs
    message_generation
    sensor_msgs
    control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Franka 0.7.0 REQUIRED)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  goal_pose.srv
  goal_joint.srv
  grasp.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    sensor_msgs
    control_msgs
  DEPENDS
    EIGEN3
    Franka
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cobot_node src/cobot_main.cpp)

add_dependencies(cobot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cobot_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(gripper_node src/cobot_gripper.cpp)

add_dependencies(gripper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gripper_node
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
)
target_include_directories(gripper_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)