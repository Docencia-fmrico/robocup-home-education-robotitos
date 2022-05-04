#include "find_my_mates/GotoArena.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/LaserScan.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "find_my_mates/BTNavAction.h"
#include <string.h>

namespace find_my_mates
{

GotoArena::GotoArena(
    const std::string& name, 
    const std::string & action_name,
    const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config)
{
    sub_laser_ = n_.subscribe("/scan",1,&GotoArena::GotoArenaCallBack,this);
    obstacle_detected_ = true;
}

void
GotoArena::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void GotoArena::GotoArenaCallBack(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    if (obstacle_detected_)
    {
        dist_ = laser->ranges[CENTER_LASER];
        obstacle_detected_ = dist_ < SECURITY_DISTANCE;
    }
}

void
GotoArena::on_halt() {}

void
GotoArena::on_start() {}

BT::NodeStatus
GotoArena::on_tick()
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
        pos_referee_.target_pose.pose.position.x = 0.0;
        pos_referee_.target_pose.pose.position.y = 0.0;
        pos_referee_.target_pose.pose.position.z = 0.0;
        pos_referee_.target_pose.pose.orientation.x = 0.0;
        pos_referee_.target_pose.pose.orientation.y = 0.0;
        pos_referee_.target_pose.pose.orientation.z = -0.267;
        pos_referee_.target_pose.pose.orientation.w = 1.0;
        set_goal(pos_referee_);
    }
    return BT::NodeStatus::RUNNING;
}

}   // namespace find_my_mates