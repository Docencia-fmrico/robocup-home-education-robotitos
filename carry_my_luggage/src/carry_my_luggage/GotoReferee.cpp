#include "carry_my_luggage/GotoReferee.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/LaserScan.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "carry_my_luggage/BTNavAction.h"
#include <string.h>

namespace carry_my_luggage
{

GotoReferee::GotoReferee(
    const std::string& name, 
    const std::string & action_name,
    const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config)
{
    sub_laser_ = n_.subscribe("/scan",1,&GotoReferee::GotoRefereeCallBack,this);
    obstacle_detected_ = true;
}

void
GotoReferee::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void GotoReferee::GotoRefereeCallBack(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    if (obstacle_detected_)
    {
        dist_ = laser->ranges[CENTER_LASER];
        obstacle_detected_ = dist_ < SECURITY_DISTANCE;
    }
}

void
GotoReferee::on_halt() {}

void
GotoReferee::on_start() {}

BT::NodeStatus
GotoReferee::on_tick()
{
    if (status() == BT::NodeStatus::IDLE)
    {
      ROS_INFO("Going to referee's position");
    }
    
    if (!obstacle_detected_)
    {
        double param;
        pos_referee_.target_pose.header.frame_id = "map";
        pos_referee_.target_pose.header.stamp = ros::Time::now();
        n_.getParam("/clase_pos/referee/x_position", param);
        pos_referee_.target_pose.pose.position.x = param;
        n_.getParam("/clase_pos/referee/y_position", param);
        pos_referee_.target_pose.pose.position.y = param;
        n_.getParam("/clase_pos/referee/z_orientation", param);
        pos_referee_.target_pose.pose.orientation.z = param;
        n_.getParam("/clase_pos/referee/w_orientation", param);
        pos_referee_.target_pose.pose.orientation.w = param;
        set_goal(pos_referee_);
    }
    return BT::NodeStatus::RUNNING;
}

}   // namespace carry_my_luggage