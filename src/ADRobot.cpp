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
 *  @file    ADRobot.cpp
 *  @authors  Aniruddh Balram, Smit Dumore, Badrinarayanan Raghunathan Srikumar
 *  @date    12/10/2022
 *  @version 0.3
 *  @brief
 *
 */

#include "anomaly_detection_robot/ADRobot.h"

/**
 * @brief Construct a new ADRobot::ADRobot object. 'navigator', 'perception'
 * classes along with state_ are also instantiated
 *
 * @param nh It is the ros node handle which contains publisher/subscribe method
 *
 */
ADRobot::ADRobot(ros::NodeHandle nh)
    : navigator(nh), perception(nh), state_(INIT) {
  ROS_INFO("Created ADRobot object");
}

/**
 * @brief This function runs on loop continuously and calls various state
 * machine parameters to move the object
 *
 */
void ADRobot::run() {
  switch (state_) {
    case INIT:
      state_ = IDLE;
      break;

    case IDLE:
      navigator.go_to_location();
      state_ = MOVING_TO_GOAL;
      break;

    case MOVING_TO_GOAL:

      std::cout << navigator.location_counter_ << "size "
                << navigator.location_size_ << "\n";

      if (navigator.navigation_status()) {
        ROS_INFO("[MOVING TO GOAL] - goal reached");
        state_ = PERCEPTION;
        if (navigator.location_counter_ == navigator.location_size_) {
          state_ = STOP;
        }
      }
      break;

    case PERCEPTION:
      if (perception.anomaly_detected(perception.image)) {
        ROS_ERROR("[ANOMALY FOUND]");

      } else {
        ROS_INFO("[NO ANOMALY]");
      }

      state_ = IDLE;

      break;

    case STOP:
      ROS_INFO("[END]");
      break;
  }
}

enum ADRobot::States ADRobot::get_state() { return state_; }
