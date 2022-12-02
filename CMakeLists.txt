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
  sensor_msgs
)


catkin_package()
#   INCLUDE_DIRS include
#   LIBRARIES anomaly_detection_robot
#   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
# )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(nav_library src/navigation.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ADRobot_node src/node.cpp)
#add_dependencies(ADRobot_node ${anomaly_detection_robot_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(ADRobot_node
  nav_library
  ${catkin_LIBRARIES}
)

###########
## Build ##
###########

#add_library(nav_library src/navigation.cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    the_rostest 
    test/all_tests.test 
    test/main.cpp)

  target_link_libraries(
    the_rostest 
    ${catkin_LIBRARIES}
    nav_library
  )
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations


