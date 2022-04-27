#ifndef ROBOCUP_DIALOG_H
#define ROBOCUP_DIALOG_H

#include "DialogInterface.h"
#include "ros/ros.h"
#include <string>

namespace gb_dialog
{
class Dialog: public DialogInterface
{
    public:
        Dialog();
        void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void detectBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void chooseBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void step();
    private:
        int state_;
        static const int IDLE = 0;
        static const int SPEAK = 1;
        static const int LISTEN = 2;
        ros::NodeHandle nh_;
};
} //namespace robocup_dialog


#endif //ROBOCUP_DIALOG_H