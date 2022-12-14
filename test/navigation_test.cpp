#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "anomaly_detection_robot/navigation.h"

geometry_msgs::PoseStamped pose_msg;

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ros::Duration(3).sleep();
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose;
}

TEST(nav_class_test, test_populate_locations) {
    ros :: NodeHandle nh;
    ros :: Subscriber test_sub = nh.subscribe("/move_base_simple/goal", 10, pose_callback);
    geometry_msgs::PoseStamped anticipate_msg;
    anticipate_msg.pose.orientation.w = 1.0;
    anticipate_msg.pose.position.z = 0.0;
    anticipate_msg.header.frame_id = "map";

    Navigation Navigation(nh);
    EXPECT_EQ(pose_msg.pose.orientation.z,anticipate_msg.pose.orientation.z);
    EXPECT_NE(pose_msg.header.frame_id, anticipate_msg.header.frame_id);

}

TEST(nav_class_test, test_nav_status) {
    ros :: NodeHandle nh;
    Navigation Navigation(nh);
    bool val = true;
    EXPECT_EQ(val, Navigation.navigation_status());
}