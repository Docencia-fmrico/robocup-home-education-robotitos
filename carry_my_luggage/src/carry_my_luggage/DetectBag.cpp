#include <carry_my_luggage/DetectBag.h>
#include <carry_my_luggage/CMLDialog.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"
#include <gb_dialog/DialogInterface.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>


#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

DetectBag::DetectBag(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, {})
{
  found_bag_ = false;
  turning_done = false;
  state_ = IDLE;
  fowarder.intent_ = "INIT";
  sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &DetectBag::DetectBagCallBack, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}


void
DetectBag::DetectBagCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes) {
  
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      if (!found_person_){
        found_person_ = true;
        px_init = (box.xmax + box.xmin) / 2;
        py_init = (box.ymax + box.ymin) / 2;
      }
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;

      if ((px - px_init) > 4 && (fowarder.intent_ == "DetectBag")){
        if ((left_counter_ >= 3))
        {
          std::cerr << "Izquierda" << std::endl;
        }
        if (!turning_done)
        {
          left_counter_++;
          right_counter_ = 0;
        }        
      }
      else if ((px - px_init < -4) && (fowarder.intent_ == "DetectBag")){
        if (right_counter_ >= 3){
          std::cerr << "Derecha" << std::endl;
        }
        if(!turning_done)
        {
          right_counter_++;
          left_counter_ = 0;
        }
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
        if ((right_counter_ >= 3) && ( fowarder.intent_ == "DetectBag")){
          
          turn_ts_ = ros::Time::now();
          state_ = TURN;
          break;
        }
        else if ((left_counter_ >= 3) && ( fowarder.intent_ == "DetectBag")){
          turn_ts_ = ros::Time::now();
          state_ = TURN;
          break;
        }
        else if (fowarder.intent_ == "GO")
        {
          ROS_INFO("[DetectBag] ---------> Follow");
          return BT::NodeStatus::SUCCESS;
          break;
        }
        state_ = LISTEN;
        break;
      case TURN:
        if (right_counter_ > 3) {
          if ((ros::Time::now() -turn_ts_).toSec() < 1){
            turning_done = true;
            cmd.angular.z = 0.3;
          }
          else {
            speak_ts_ = ros::Time::now();
            turning_done = true;
            cmd.angular.z = 0;
            fowarder.intent_ = "GO";
            state_ = SPEAK;
          }
        }
        else if (left_counter_ > 3) {
          if ((ros::Time::now() -turn_ts_).toSec() < 3){
            cmd.angular.z = -0.3;
            turning_done = true;

          }
          else {
            speak_ts_ = ros::Time::now();
            turning_done = true;
            cmd.angular.z = 0;
            fowarder.intent_ = "GO";
            state_ = SPEAK;
          }
        }
        pub_vel_.publish(cmd);
        break;
      case LISTEN:
        if ( fowarder.intent_ == "DetectBag") {
         

          state_ = IDLE;
          break;
        }
        speak_ts_ = ros::Time::now();
        cmd.angular.z = 0.0;
        state_ = SPEAK;
        break;
      case SPEAK:
        
        if (fowarder.intent_ == "INIT") {
          fowarder.speak("Choose one bag");
          fowarder.intent_ = "DetectBag";
        }
        if (fowarder.intent_ == "GO") {
          fowarder.speak("Ok, I follow you now");
          fowarder.intent_ == "DetectBag";
        }
        if ((ros::Time::now() - speak_ts_).toSec() >= 3)
        {
          state_ = IDLE;
        }
        break;
    }
    return BT::NodeStatus::RUNNING;
    
}

}  // namespace carry_my_luggage