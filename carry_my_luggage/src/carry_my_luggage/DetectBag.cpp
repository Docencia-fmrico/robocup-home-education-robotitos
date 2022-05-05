#include <carry_my_luggage/DetectBag.h>
#include <carry_my_luggage/CMLDialog.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <gb_dialog/DialogInterface.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ros/ros.h"
#include <string>

#define PIXEL_REQ 1000

namespace carry_my_luggage
{

DetectBag::DetectBag(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, {})
{
  found_bag_ = false;
  pixel_counter_ = 0;
  right_counter_ = 0;
  left_counter_ = 0;
  sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &DetectBag::DetectBagCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}


void
DetectBag::DetectBagCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes) {
  
  turn_ts_ = ros::Time::now();
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      if (!found_person_){
        found_person_ = true;
        px_init = (box.xmax + box.xmin) / 2;
        py_init = (box.ymax + box.ymin) / 2;
      }
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;

      if (px - px_init > 4){
        if (left_counter_ >= 3)
        {
          std::cerr << "Izquierda" << std::endl;
        }
        left_counter_++;
        right_counter_ = 0;
        
      }
      else if (px - px_init < -4){
        if (right_counter_ >= 3){
          std::cerr << "Derecha" << std::endl;
          turn_ts_ = ros::Time::now();
        }
        right_counter_++;
        left_counter_ = 0;
      }
    }
  }
}

void
DetectBag::halt()
{
  ROS_INFO("DetectBag halt");
}

BT::NodeStatus
DetectBag::tick()
{
  dialogflow_ros_msgs::DialogflowResult result;

  switch (state_)
    {
      case IDLE:
        if (fowarder.intent_ == "Default Fallback Intent")
        {
          state_ = LISTEN;
          break;
        }
        if (right_counter_ >= 3){
          turn_ts_ = ros::Time::now();
          state_ = TURN;
          break;
        }
        else if (left_counter_ >= 3){
          turn_ts_ = ros::Time::now();
          state_ = TURN;
          break;
        }
        state_ = LISTEN;
        break;
      case TURN:
        if (right_counter_ >= 3) {
          if ((ros::Time::now() - turn_ts_).toSec() >= 3) {
            cmd.angular.z = -0.3;
          }
          else {
            cmd.angular.z = 0.0;
          }
        }
        if (left_counter_ >= 3) {
          if ((ros::Time::now() - turn_ts_).toSec() >= 3) {
            cmd.angular.z = 0.3;
          }
          else {
            cmd.angular.z = 0.0;
          }
        }
        pub_vel_.publish(cmd);
        state_ = LISTEN;
        break;
      case LISTEN:
        //fowarder.listen();
        //speak_ts_ = ros::Time::now();
        state_ = SPEAK;
        break;
      case SPEAK:
        //if ((ros::Time::now() - speak_ts_).toSec() >= 3)
        //{
        state_ = IDLE;
        //}
        break;
    }
  
  /*if (left_counter_ >= 4) {
    while ((ros::Time::now() - turn_ts_).toSec() <= 3)
    {
      std::cerr << "ESTOY AQUI IZQUIERDAA" << std::endl;
      cmd.angular.z = 0.3;
      pub_vel_.publish(cmd);
    }
  }
  else if (right_counter_ >= 4) {
    while ((ros::Time::now() - turn_ts_).toSec() <= 3)
    {
      std::cerr << (ros::Time::now() -turn_ts_).toSec() << std::endl;
      std::cerr << "ESTOY AQUI DERECHA" << std::endl;
      cmd.angular.z = -0.3;
      pub_vel_.publish(cmd);
    }
  }
  cmd.angular.z = 0.0;
  pub_vel_.publish(cmd);
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Bag");
  }*/

  return BT::NodeStatus::SUCCESS;
}

}  // namespace carry_my_luggage