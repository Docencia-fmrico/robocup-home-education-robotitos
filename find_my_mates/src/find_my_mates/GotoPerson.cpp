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
: BTNavAction(name, action_name, config), goal_sent(false), count(0)
{ 
  direction_= n_.subscribe("/amcl_pose", 1, &GotoPerson::DirectionCallBack,this);
  //goal_= n_.subscribe("/move_base/status", 1, &GotoPerson::statusCallBack,this);
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
GotoPerson::on_halt() {
}

void
GotoPerson::on_start() {}

BT::NodeStatus
GotoPerson::on_tick()
{
  move_base_msgs::MoveBaseGoal goal;
  for (int i =0; i<6; i++) {
    if (person[i][0] == directions.target_pose.pose.position.x) {
      count = i++;
      std::cout << count << std::endl;
      goal_sent = false;
    }
  }
  
  if (!goal_sent) {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = person[count][0];
    goal.target_pose.pose.position.y = person[count][1];
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = person[count][2];
    goal.target_pose.pose.orientation.w = person[count][3];
    std::cout <<"x: " << person[count][0] << ", y: " << person[count][1] << ", z: " << person[count][2] << std::endl;

    set_goal(goal);
    goal_sent = true;
  } 
  return BT::NodeStatus::RUNNING;
}

}  // namespace find_my_mates