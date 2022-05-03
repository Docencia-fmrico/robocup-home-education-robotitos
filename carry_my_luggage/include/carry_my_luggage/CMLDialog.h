#ifndef CARRY_MY_LUGGAGE_CML_DIALOG_H
#define CARRY_MY_LUGGAGE_CML_DIALOG_H

#include <gb_dialog/DialogInterface.h>
#include "ros/ros.h"
#include <string>

namespace ph = std::placeholders;

namespace carry_my_luggage
{

class Dialog : public gb_dialog::DialogInterface
{
    public:
        explicit Dialog();
        std::string getContext(dialogflow_ros_msgs::DialogflowResult result);
        std::string getIntent(dialogflow_ros_msgs::DialogflowResult result);
        void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void detectBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void chooseBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void step();
    private:
        int state_;
        std::string context_;
        std::string intent_;
        static const int IDLE = 0;
        static const int SPEAK = 1;
        static const int LISTEN = 2;
        ros::Time speak_ts_;
        ros::NodeHandle nh_;
};

} //namespace carry_my_luggage


#endif // CARRY_MY_LUGGAGE_CML_DIALOG_H