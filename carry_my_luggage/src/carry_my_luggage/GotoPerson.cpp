#include "carry_my_luggage/GotoPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/message_filter.h"

#include "ros/ros.h"
#include <string>
#include "carry_my_luggage/transforms.h"

namespace carry_my_luggage
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

  move_base_msgs::MoveBaseGoal goal;
  
  
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2person_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2person_msg, bf2person);
    
    double dist = bf2person.getOrigin().length();
    double angle = atan2(bf2person.getOrigin().y(), bf2person.getOrigin().x());
    
    //es la forma de obtener los valores de los ejes de rotacion
    //Se imprime los valores obtenidos antes de las coordenadas
    ROS_INFO("base_footprint -> person [%lf, %lf] dist=%lf angle=%lf %lf ago", bf2person.getOrigin().x(), bf2person.getOrigin().y(), dist, angle, (ros::Time::now() - bf2person.stamp_).toSec());
    
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
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace carry_my_luggage