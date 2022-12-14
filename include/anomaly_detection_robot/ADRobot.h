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
 *  @file    ADRobot.h
 *  @author  Smit Dumore
 *  @date    11/30/2022
 *  @version 0.1
 *  @brief  
 *
 */

#pragma once
#include <ros/ros.h>
#include "anomaly_detection_robot/navigation.h"
#include "anomaly_detection_robot/detect_anomaly.h"
#include "anomaly_detection_robot/perception.h"

/**
 * @brief ADRobot class
 * 
 */
class ADRobot {
 public:
    explicit ADRobot(ros::NodeHandle);
    void run();

    enum States {
        INIT,
        IDLE,
        MOVING_TO_GOAL,
        STOP,
        PERCEPTION
    };

 private:
    Navigation navigator;
    Detect_anomaly detectAnomaly;

    States state_;
    ros::NodeHandle* nh_;

};