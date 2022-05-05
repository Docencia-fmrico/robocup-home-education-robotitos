#include "carry_my_luggage/GotoPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

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
: BTNavAction(name, action_name, config), listener(buffer), counter_(0)
{ 
  direction_= n_.subscribe("/amcl_pose", 1, &GotoPerson::DirectionCallBack,this);
  sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &GotoPerson::GotoPersonCallBack,this);
}

void
GotoPerson::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

void
GotoPerson::GotoPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
  ROS_INFO(" callback person");
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      px = (box.xmax + box.xmin) / 2;
    } 
  }
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

  if (counter_++ == 10)
  {
    if (buffer.canTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1), &error))
    {
      map2bf_msg = buffer.lookupTransform("map", "base_footprint", ros::Time(0));

      tf2::fromMsg(map2bf_msg, map2bf);
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    if (buffer.canTransform("base_footprint", "odom", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2odom_msg = buffer.lookupTransform("base_footprint", "odom", ros::Time(0));

      tf2::fromMsg(bf2odom_msg, bf2odom);
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    if (buffer.canTransform("odom", "object/0", ros::Time(0), ros::Duration(0.1), &error))
    {
      odom2obj_msg = buffer.lookupTransform("odom", "object/0", ros::Time(0));

      tf2::fromMsg(odom2obj_msg, odom2obj);
    }
    else
    {
      std::cout << "fuera del alcance" << std::endl;
      ROS_ERROR("%s", error.c_str());
    }
    
    double dist = bf2obj.getOrigin().length();
    bf2obj = map2bf * bf2odom * odom2obj;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation.x = directions.target_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = directions.target_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = directions.target_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = directions.target_pose.pose.orientation.w;

    if (dist > 1) {
      goal.target_pose.pose.position.x = bf2obj.getOrigin().x();
      goal.target_pose.pose.position.y = bf2obj.getOrigin().y();
      goal.target_pose.pose.position.z = bf2obj.getOrigin().z();
    } else {
      goal.target_pose.pose.position.x = directions.target_pose.pose.position.x;
      goal.target_pose.pose.position.y = directions.target_pose.pose.position.y;
      goal.target_pose.pose.position.z = directions.target_pose.pose.position.z;
    }

    set_goal(goal);
    counter_ = 0;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace carry_my_luggage