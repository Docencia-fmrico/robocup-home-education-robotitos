#include "find_my_mates/GotoPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "find_my_mates/BTNavAction.h"

#include "ros/ros.h"
#include <string>
#include "find_my_mates/transforms.h"

namespace find_my_mates
{

GotoPerson::GotoPerson(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), listener(buffer)
{ 
  direction_= n_.subscribe("/amcl_pose", 1, &GotoPerson::DirectionCallBack,this);
}

void
GotoPerson::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void GotoPerson::DirectionCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position) {
  directions.target_pose.header.stamp = position.get()->header.stamp;
  directions.target_pose.pose.position.x = position.get()->pose.pose.position.x;
  directions.target_pose.pose.position.y = position.get()->pose.pose.position.y;
  directions.target_pose.pose.position.z = position.get()->pose.pose.position.z;

  directions.target_pose.pose.orientation.x = position.get()->pose.pose.orientation.x;
  directions.target_pose.pose.orientation.y = position.get()->pose.pose.orientation.y;
  directions.target_pose.pose.orientation.z = position.get()->pose.pose.orientation.z;
  directions.target_pose.pose.orientation.w = position.get()->pose.pose.orientation.w;
}

void
GotoPerson::on_halt() {}

void
GotoPerson::on_start() {}

BT::NodeStatus
GotoPerson::on_tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Siguiendo a la persona");
  }

  /*
  move_base_msgs::MoveBaseGoal goal;
  
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = bf2person.getOrigin().x();
    goal.target_pose.pose.position.y = bf2person.getOrigin().y();
    goal.target_pose.pose.position.z = bf2person.getOrigin().z();
    goal.target_pose.pose.orientation.x = directions.target_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = directions.target_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = directions.target_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = directions.target_pose.pose.orientation.w;

    set_goal(goal);

  } else {

  }*/
  return BT::NodeStatus::SUCCESS;
}

}  // namespace find_my_mates