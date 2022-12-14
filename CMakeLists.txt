cmake_minimum_required(VERSION 3.0.2)
project(anomaly_detection_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)


catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(adr_library src/ADRobot.cpp)
add_library(nav_library src/navigation.cpp)
add_library(perception_library src/perception.cpp)
add_library(anomaly_library src/detect_anomaly.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ADRobot_node src/node.cpp)
#add_dependencies(ADRobot_node ${anomaly_detection_robot_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(ADRobot_node
  adr_library  
  nav_library
  perception_library
  anomaly_library
  ${catkin_LIBRARIES}
)

###########
## Build ##
###########

#add_library(nav_library src/navigation.cpp)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(
#     the_rostest 
#     test/all_tests.test 
#     test/main.cpp)

#   target_link_libraries(
#     the_rostest 
#     ${catkin_LIBRARIES}
#     nav_library
#     perception_library
#     adr_library
#     anomaly_library
#   )
# endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations

