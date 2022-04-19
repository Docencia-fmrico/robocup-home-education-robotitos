#include "carry_my_luggage/GotoPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"

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

  if (status() == BT::NodeStatus::IDLE)
  {
      ROS_INFO("Loking for a ball dist");
  }
  
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2ball_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2ball_msg, bf2ball);
    
    double dist = bf2ball.getOrigin().length();
    double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());

    //es la forma de obtener los valores de los ejes de rotacion
    //Se imprime los valores obtenidos antes de las coordenadas
    ROS_INFO("base_footprint -> ball [%lf, %lf] dist=%lf angle=%lf %lf ago", bf2ball.getOrigin().x(), bf2ball.getOrigin().y(), dist, angle, (ros::Time::now() - bf2ball.stamp_).toSec());
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = bf2ball.getOrigin().x;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

  } else {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace carry_my_luggage