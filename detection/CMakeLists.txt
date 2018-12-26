cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
project(detection)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
  jsk_recognition_msgs
  tracking
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  PostsPositions.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
#  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detection
  CATKIN_DEPENDS roscpp rospy message_runtime jsk_recognition_msgs tracking
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#include_directories(src ${EIGEN3_INCLUDE_DIR})

add_library(bounding_boxes src/bounding_boxes.cpp)
add_dependencies(bounding_boxes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bounding_boxes_node src/bounding_boxes_node.cpp)
target_link_libraries(bounding_boxes_node bounding_boxes ${catkin_LIBRARIES})
add_dependencies(bounding_boxes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(matching src/matching.cpp)
add_dependencies(matching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(matching_node src/matching_node.cpp)
target_link_libraries(matching_node matching ${catkin_LIBRARIES})
add_dependencies(matching_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(d2p tests/tests_d2p.cpp)
target_link_libraries(d2p bounding_boxes ${catkin_LIBRARIES})