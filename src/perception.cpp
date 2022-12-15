/********************************************************************
 * Copyright (c) 2022 Aniruddh Balram
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
 *  @file    perception.cpp
 *  @author  Aniruddh Balram
 *  @date    12/10/2022
 *  @version 0.3
 *  @brief  
 *
 */

#include "anomaly_detection_robot/perception.h"


Perception::Perception(ros::NodeHandle nh) : it(nh) {
  ROS_INFO("Perception object created");
  sub = it.subscribe("/camera/rgb/image_raw", 1, &Perception::camera_callback, this); 
}

void Perception::camera_callback(const sensor_msgs::ImageConstPtr& msg) {
    // recieve images from camera
    try{
        input_image = cv_bridge::toCvShare(msg,"bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool Perception::anomaly_detected() {
   // Create a gray scale image
   cv::Mat gray_img; 
   cv::cvtColor(input_image, gray_img, cv::COLOR_BGR2GRAY);
   // Now what needs to be done is apply binary thresholding so that the image only has 0 and 255 pixel intensity
   cv::Mat thresholded_image;
   cv::Mat blur;
   cv::GaussianBlur(gray_img, blur, cv::Size(5,5), 0);
   cv::threshold(blur, thresholded_image, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU); // Use Otsus binary thresholding
   cv::findContours(thresholded_image, contours, hierarchy_of_contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
   if(contours.size() > 1) {
      return true;
   }
   else {
      return false;
   }
}