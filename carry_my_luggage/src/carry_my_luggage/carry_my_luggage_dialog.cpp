/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics Labs
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jaime Avilleira j.avilleira.2019@alumnos.urjc.es */

/* Mantainer: Jaime Avilleira j.avilleira.2019@alumnos.urjc.es*/
#include "gb_dialog/DialogInterface.h"
#include "gb_dialog/robocup_dialog.h"
#include <string>
#include <iostream>

namespace ph = std::placeholders;
namespace gb_dialog
{

Dialog::Dialog(): nh_()
{
    this->registerCallback(std::bind(&Dialog::noIntentCB, this, ph::_1));
    this->registerCallback(
        std::bind(&Dialog::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
    this->registerCallback(
        std::bind(&Dialog::detectBagIntentCB, this, ph::_1),
        "DetectBag");
    this->registerCallback(
        std::bind(&Dialog::chooseBagIntentCB, this, ph::_1),
        "Choose Bag");
}

std::string
Dialog::getContext(dialogflow_ros_msgs::DialogflowResult result)
{
    for (const auto & param : result.contexts) {
        context_ = param.name;
        context_ = context_.substr(87);
        std::cerr << "Param: " << context_ << std::endl;
    }
    return context_;
}

std::string
Dialog::getIntent(dialogflow_ros_msgs::DialogflowResult result)
{
    intent_ = result.intent.c_str();
    ROS_INFO("ueue[%s]", result.intent.c_str());
    return intent_;
}


void
Dialog::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
    ROS_INFO("[Dialog] noIntentCB: intent [%s]", result.intent.c_str());
    getIntent(result);
    speak(result.fulfillment_text);
}

void 
Dialog::welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
  ROS_INFO("[------------------Dialog] welcomeIntentCB: intent [%s]", result.intent.c_str());
  speak(result.fulfillment_text);
}

void
Dialog::detectBagIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
    getIntent(result);
    ROS_INFO("[Dialog] detectBagIntentCB: intent [%s]", result.intent.c_str());
    getContext(result);
    speak(result.fulfillment_text);
}

void
Dialog::chooseBagIntentCB(dialogflow_ros_msgs::DialogflowResult result)
{
    ROS_INFO("[Dialog] chooseBagIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
}

void
Dialog::step()
{
    dialogflow_ros_msgs::DialogflowResult result;
    switch (state_)
    {
        case IDLE:
            if (intent_ == "Default Fallback Intent")
            {
                state_ = LISTEN;
                std::cerr << "tugatita" << std::endl;
                break;
            }
            state_ = LISTEN;
            break;
        case LISTEN:
            listen();
            speak_ts_ = ros::Time::now();
            state_ = SPEAK;
            break;
        case SPEAK:
            if ((ros::Time::now() - speak_ts_).toSec() >= 3)
            {
                state_ = IDLE;
            }
            break;
    }
}



} //namespace robocup_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robocup_dialog_node");
  gb_dialog::Dialog forwarder;
  ros::Rate loop_rate(20);
  while (ros::ok()){
      forwarder.step();
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}