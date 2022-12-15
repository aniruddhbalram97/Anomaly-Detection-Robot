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
 *  @file    navigation.cpp
 *  @author  Smit Dumore
 *  @date    11/30/2022
 *  @version 0.1
 *  @brief  
 *
 */

#include "anomaly_detection_robot/navigation.h"

/**
 * @brief Construct a new Navigation:: Navigation object
 * 
 */
Navigation::Navigation(ros::NodeHandle nh) {

    ROS_INFO("Navigation object created");

    location_counter_ = -1;

    move_base_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
                                        "/move_base_simple/goal", 10);
    
    pose_sub_ = nh.subscribe("/odom", 10, &Navigation::pose_callback, this);

    populate_locations();
}

/**
 * @brief Send robot to location
 * 
 */
void Navigation::go_to_location() {
    //function to move to location

    location_counter_++;

    if(location_counter_ < map_locations.size()) {
        geometry_msgs::PoseStamped curr_goal = map_locations[location_counter_];
        move_base_goal_pub_.publish(curr_goal);
        goal_pose_ = curr_goal;
        ROS_INFO("New Goal Published");
    }else{
        ROS_ERROR("no goals left");
    }
}

/**
 * @brief Functions to populate locations to the robot
 * 
 */

void Navigation::populate_locations() {
    std::vector<double> map_x = {0.5, 0.5, -0.5, -0.5, -1.0};
    std::vector<double> map_y = {0.5, -0.5, -0.5, 0.5, -0.5};
    int size = map_x.size();

    for(int i=0; i < size; i++) {
        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.frame_id = "map";
        curr_pose.pose.position.x = map_x[i];
        curr_pose.pose.position.y = map_y[i];
        curr_pose.pose.orientation.w = 1.0;
        curr_pose.pose.orientation.x = 0.0;
        curr_pose.pose.orientation.y = 0.0;
        curr_pose.pose.orientation.z = 0.0;
        map_locations.push_back(curr_pose);
    }

    ROS_INFO("Locations populated");
}

/**
 * @brief Check if goal is reached
 * 
 * @return true 
 * @return false 
 */
bool Navigation::navigation_status() {

    double x_sq = std::pow(curr_pose_.position.x - goal_pose_.pose.position.x, 2);
    
    double y_sq = std::pow(curr_pose_.position.y - goal_pose_.pose.position.y, 2);

    double distance = std::sqrt(x_sq + y_sq);
    
    if (distance <= 0.1) return true;
    return false;
}

/**
 * @brief callback function for subscriber
 * 
 * @param pose 
 */
void Navigation::pose_callback(const nav_msgs::Odometry &pose) {
    curr_pose_ = pose.pose.pose;
    //ROS_INFO("POSE X .. %f", curr_pose_.position.x);
    //ROS_INFO("POSE Y .. %f", curr_pose_.position.y);
}
