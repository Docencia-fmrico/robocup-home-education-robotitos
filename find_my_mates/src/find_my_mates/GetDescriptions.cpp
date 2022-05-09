#include <gb_dialog/DialogInterface.h>
#include <find_my_mates/GetDescriptions.h>
#include <string>
#include <iostream>

namespace ph = std::placeholders;
namespace find_my_mates
{

Dialog::Dialog()
{
    this->registerCallback(std::bind(&Dialog::noIntentCB, this, ph::_1));
    this->registerCallback(
        std::bind(&Dialog::personNameCB, this, ph::_1),
        "person_name");
    this->registerCallback(
        std::bind(&Dialog::clothesColorCB, this, ph::_1),
        "clothes_color");
    this->registerCallback(
        std::bind(&Dialog::personObjectCB, this, ph::_1),
        "person_object");
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
Dialog::personNameCB(dialogflow_ros_msgs::DialogflowResult result)
{
    ROS_INFO("[Dialog] personNameCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    for(const auto & param : result.parameters){
        for (const auto & value : param.value){
          person[0] = value;
        }
      }
    ROS_INFO("[QUESTION] What is the color of your clothes?");
}

void
Dialog::clothesColorCB(dialogflow_ros_msgs::DialogflowResult result)
{
    ROS_INFO("[Dialog] clothesColorCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    for(const auto & param : result.parameters){
        for (const auto & value : param.value){
          person[1] = value;
        }
    }
    ROS_INFO("[QUESTION] What object do you carry?");
}

void
Dialog::personObjectCB(dialogflow_ros_msgs::DialogflowResult result)
{
    ROS_INFO("[Dialog] personObjectCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    for(const auto & param : result.parameters){
        for (const auto & value : param.value){
          person[2] = value;
        }
      }
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
            if (person_cnt_ != cnt_aux_)
            {
              speak("Hi, what is your name?");
              cnt_aux_ = person_cnt_;
              ROS_INFO("[QUESTION] What is your name?");
            }
            if ((ros::Time::now() - speak_ts_).toSec() >= 3)
            {
                state_ = IDLE;
            }
            break;
    }
}

} // namespace find_my_mates