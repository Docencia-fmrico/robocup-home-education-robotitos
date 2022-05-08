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
: BTNavAction(name, action_name, config), counter_(0)
{
    sub_laser_ = n_.subscribe("/scan",1,&GotoReferee::GotoRefereeCallBack,this);
    obstacle_detected_ = true;
    goal_sent = false;
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
GotoReferee::on_start() {
    pos_referee_.target_pose.header.frame_id = "map";
    pos_referee_.target_pose.header.stamp = ros::Time::now();
    pos_referee_.target_pose.pose.position.x = 3.345;
    pos_referee_.target_pose.pose.position.y = 2.922;
    pos_referee_.target_pose.pose.position.z = 0.0;
    pos_referee_.target_pose.pose.orientation.x = 0.0;
    pos_referee_.target_pose.pose.orientation.y = 0.0;
    pos_referee_.target_pose.pose.orientation.z = 3.03;
    pos_referee_.target_pose.pose.orientation.w = 1.0;
    set_goal(pos_referee_);
    goal_sent = true;
}

BT::NodeStatus
GotoReferee::on_tick()
{
    if (status() == BT::NodeStatus::IDLE)
    {
      ROS_INFO("Going to referee's position");
    }
    
    if (counter_++ == 200)
    {
        pos_referee_.target_pose.header.frame_id = "map";
        pos_referee_.target_pose.header.stamp = ros::Time::now();
        pos_referee_.target_pose.pose.position.x = 1.12;
        pos_referee_.target_pose.pose.position.y = 3.38;
        pos_referee_.target_pose.pose.position.z = 0.0;
        pos_referee_.target_pose.pose.orientation.x = 0.0;
        pos_referee_.target_pose.pose.orientation.y = 0.0;
        pos_referee_.target_pose.pose.orientation.z = -2.452;
        pos_referee_.target_pose.pose.orientation.w = 1.0;
        set_goal(pos_referee_);
    }

    return BT::NodeStatus::RUNNING;
}

}   // namespace carry_my_luggage