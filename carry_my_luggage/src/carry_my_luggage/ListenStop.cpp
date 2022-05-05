#include <gb_dialog/DialogInterface.h>
#include <carry_my_luggage/ListenStop.cpp>
#include <string>
#include <iostream>

namespace ph = std::placeholders;
namespace carry_my_luggage {

    Dialog::Dialog()
    {
        this->registerCallback(std::bind(&Dialog::noIntentCB, this, ph::_1));
        this->registerCallback(
            std::bind(&Dialog::goBackCB, this, ph::_1),
            "ReturntoArena");
    }

    void
    Dialog::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
        ROS_INFO("[Dialog] noIntentCB: intent [%s]", result.intent.c_str());
        speak(result.fulfillment_text);
    }

    void
    Dialog::goBackCB(dialogflow_ros_msgs::DialogflowResult result)
    {
        ROS_INFO("[Dialog] goBackCB: intent [%s]", result.intent.c_str());
        speak(result.fulfillment_text);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace carry_my_luggage