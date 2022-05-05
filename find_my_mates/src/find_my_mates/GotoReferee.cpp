#include "find_my_mates/GotoReferee.h"
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

GotoReferee::GotoReferee(
    const std::string& name, 
    const std::string & action_name,
    const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config)
{
    
}

void
GotoReferee::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void
GotoReferee::on_halt() {
}

void
GotoReferee::on_start() {
  pos_referee_.target_pose.header.frame_id = "map";
  pos_referee_.target_pose.header.stamp = ros::Time::now();
  pos_referee_.target_pose.pose.position.x = 4.994;
  pos_referee_.target_pose.pose.position.y = -1.018;
  pos_referee_.target_pose.pose.position.y = 0.0;
  pos_referee_.target_pose.pose.orientation.x = 0.0;
  pos_referee_.target_pose.pose.orientation.y = 0.0;
  pos_referee_.target_pose.pose.orientation.z = -0.527;
  pos_referee_.target_pose.pose.orientation.w = 1.0;
  set_goal(pos_referee_);
}

BT::NodeStatus
GotoReferee::on_tick()
{
    if (status() == BT::NodeStatus::IDLE)
    {
      ROS_INFO("Going to referee's position");
    }
    return BT::NodeStatus::RUNNING;
}

}   // namespace find_my_mates