/********************************************************************
 * Copyright (c) 2022 Smit Dumore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/**
 * @file navigation_test.cpp
 * @author Aniruddh Balram, Smit Dumore, Badrinarayanan Raghunathan Srikumar
 * @brief Test blocks to test navigation class
 * @version 0.1
 * @date 2022-12-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "anomaly_detection_robot/navigation.h"
/**
 * @brief Callback function
 * 
 */

geometry_msgs::PoseStamped pose_msg;

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ros::Duration(3).sleep();
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose;
}

/**
 * @brief TEST stub to check populate_location function
 * 
 */

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

/**
 * @brief TEST stub to check nav_status function
 * 
 */
TEST(nav_class_test, test_nav_status) {
    ros :: NodeHandle nh;
    Navigation Navigation(nh);
    bool val = true;
    EXPECT_EQ(val, Navigation.navigation_status());
}

TEST(nav_class_test, test_go_to_location) {
    ros :: NodeHandle nh;
    Navigation Navigation(nh);
    EXPECT_NO_THROW(Navigation.go_to_location());
}
