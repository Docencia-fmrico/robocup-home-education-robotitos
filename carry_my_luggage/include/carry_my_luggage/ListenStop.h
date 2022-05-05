#ifndef CARRY_MY_LUGGAGE_LISTENSTOP_H
#define CARRY_MY_LUGGAGE_LISTENSTOP_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <gb_dialog/DialogInterface.h>
#include "ros/ros.h"
#include <string>


namespace ph = std::placeholders;

namespace carry_my_luggage
{
    class Dialog : public gb_dialog::DialogInterface
{
    public:
        Dialog();
        void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
        void goBackCB(dialogflow_ros_msgs::DialogflowResult result);
        void step();
    private:
        ros::NodeHandle nh_;

} // namespace carry_my_luggage
#endif  // CARRY_MY_LUGGAGE_GOTOARENA_H 
