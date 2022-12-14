#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "anomaly_detection_robot/ADRobot.h"

TEST(ADRobot_class_test, init_state_test) {
    ros::NodeHandle nh;
    ADRobot ADR(nh);
    enum States {
        INIT
    };
    States robot_state;
    EXPECT_EQ(robot_state,ADR.get_state());

}