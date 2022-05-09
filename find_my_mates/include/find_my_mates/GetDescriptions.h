#ifndef FIND_MY_MATES_GET_DESCRIPTIONS_H
#define FIND_MY_MATES_GET_DESCRIPTIONS_H

#include <gb_dialog/DialogInterface.h>
#include "ros/ros.h"
#include <string>

namespace ph = std::placeholders;

namespace find_my_mates
{
class Dialog : public gb_dialog::DialogInterface
{
    public:
        Dialog();
        std::string intent_;
        std::string person[3];
        std::string getContext(dialogflow_ros_msgs::DialogflowResult result);
        std::string getIntent(dialogflow_ros_msgs::DialogflowResult result);
        void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void personNameCB(dialogflow_ros_msgs::DialogflowResult result);
        void clothesColorCB(dialogflow_ros_msgs::DialogflowResult result);
        void personObjectCB(dialogflow_ros_msgs::DialogflowResult result);
        void step();
    private:
        int state_ = 0;
        int person_cnt_ = 1;
        int cnt_aux_ = 0;
        std::string context_;
        static const int IDLE = 0;
        static const int SPEAK = 1;
        static const int LISTEN = 2;
        ros::Time speak_ts_;
        ros::NodeHandle nh_;
};

} //namespace find_my_mates


#endif // FIND_MY_MATES_GET_DESCRIPTIONS_H