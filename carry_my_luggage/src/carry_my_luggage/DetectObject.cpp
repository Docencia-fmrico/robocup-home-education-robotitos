#include "carry_my_luggage/DetectObject.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

DetectObject::DetectObject(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), obstacle_detected_(false), state_(GOING_FORWARD)
{ 
  sub_laser_ = n_.subscribe("/scan_filtered",1,&DetectObject::laserCallBack,this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}

void
DetectObject::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  int left = 700;
  int right = 150;
  
  right_dist = laser->ranges[right];
  left_dist = laser->ranges[left];
  
  if (left_dist < SECURITY_DISTANCE)
  {
    obstacle_state_ = LEFT_DETECTED;
    obstacle_detected_ = true;
  }
  else if (right_dist < SECURITY_DISTANCE)
  {
    obstacle_state_ = RIGHT_DETECTED;
    obstacle_detected_ = true;
  }
  else
  {
    obstacle_detected_ = false;
  }
}

BT::NodeStatus
DetectObject::tick()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
     case GOING_FORWARD:
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      if (obstacle_detected_)
      {
        detected_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;
    case GOING_BACK:
      cmd.linear.x = -0.0;
      cmd.angular.z = 0.0;

      if ((ros::Time::now() - detected_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        if (obstacle_state_ == RIGHT_DETECTED)
        {
          state_ = TURNING_LEFT;
          ROS_INFO("GOING_BACK -> TURNING_LEFT");
        }
        else
        {
          state_ = TURNING_RIGHT;
          ROS_INFO("GOING_BACK -> TURNING_RIGHT");
        }
      }

      break;
    case TURNING_LEFT:
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;


      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_LEFT -> GOING_FORWARD");
      }
      break;
    case TURNING_RIGHT:
      cmd.linear.x = 0.0;
      cmd.angular.z = -0.0;


      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_RIGHT -> GOING_FORWARD");
      }
      break;
    }

  pub_vel_.publish(cmd);
  if (state_ != GOING_FORWARD) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace carry_my_luggage