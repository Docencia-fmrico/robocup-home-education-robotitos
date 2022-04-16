#include "carry_my_luggage/GotoPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

GotoPerson::GotoPerson(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config)
{ 
}

void
GotoPerson::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void
GotoPerson::on_halt()
{
  ROS_INFO("FollowPerson halt");
}

void
GotoPerson::on_start()
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  set_goal(goal);

  ROS_INFO("Move start");
}

BT::NodeStatus
GotoPerson::on_tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }


  return BT::NodeStatus::SUCCESS;
}

}  // namespace carry_my_luggage