[![Build Status](https://github.com/Irdab2000/Anomaly-Detection-Robot/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/aniruddhbalram97/Anomaly-Detection-Robot/actions/workflows/build_and_coveralls.yml)
[![Coverage Status](https://coveralls.io/repos/github/Irdab2000/Anomaly-Detection-Robot/badge.svg?branch=master)](https://coveralls.io/github/Irdab2000/Anomaly-Detection-Robot?branch=master)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Anomaly Detection Robot

### Overview
- The aim of this project is to survey an environment using turtlebot3 and find anomalies and it's locations.

### Dependencies/ Assumptions
- OS : Ubuntu 20.04
- ROS Distro : Noetic
- Package build type : ```catkin_make```
- Package dependencies : ```catkin```, ```std_msgs```, ```geometry_msgs```, ```sensor_msgs```, ```nav_msgs```

We expect you to have gazebo packages already installed in your pc. If not follow the following steps
```
cd <your ROS_ws>/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

### AIP Sheet
- Product Backlog Sheet [google sheet](https://docs.google.com/spreadsheets/d/1uHDDDbMvHY4QyDjH6F0JqCa6SoQ2CfuYaCJu3RNxHEI/edit#gid=438129087)
- Sprint Planning Notes [google doc](https://docs.google.com/document/d/1wdccIWWXtUxuXXT_2JpvLoirFL0By40nLpI3DHsvV1o/edit)

### Presentation link
https://docs.google.com/presentation/d/1BKFK73goBxVEZzWt95vCPfu6BDekYOlmda1Gdb-Nc90/edit?usp=sharing

### Demo video
https://drive.google.com/file/d/18zbLacLETwxDgiNdPukttsvHfr0zEX-X/view?usp=share_link

## Steps to run the ROS package
### Build instructions
```
cd <your ROS_ws>/src
source ~/devel/setup.bash
git clone https://github.com/aniruddhbalram97/Anomaly-Detection-Robot.git
cd ..
catkin_make
```

### Launch the world
- To launch the world, open a new terminal and run:
```
cd <Your ROS_ws>
source ~/devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
- Make sure that, the launch file is pointing to the world that has been provided in this repository

### Launch the navigation stack and map for this world
- Open a new terminal and run
```
cd <Your ROS_ws>
source ~/devel/setup.bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
### Launch the node, that performs the task
- Open a new terminal
```
cd <your ROS_ws>
source ~/devel/setup.bash
rosrun anomaly-detection-robot ADRobot_node
```
### ROS Build TESTS
- Open a new terminal
```
cd <your ROS_ws>
source ~/devel/setup.bash
catkin_make tests
```
### ROS Run TESTS
- Open a new terminal
```
cd <your ROS_ws>
rostest anomaly_detection_robot all_tests.test
```
### Generating Documentation
- Run the following command in folder's root directory to generate new documentation

```
doxygen docs/doxygen_config.conf
```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```
### Google Styling format
Run the following command from the directory where the .cpp files are present(src in this case)
```
clang-format -style=Google -i your_file.cpp
```

### Known Issue
move_base fails to generate valid paths when the obstruction is very close to the Robot
