#include "carry_my_luggage/DetectBag.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

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
  sub_hsv_ = n_.subscribe("/hsv/image_filtered",1,&DetectBag::DetectBagCallBack,this);
}

void
DetectBag::DetectBagCallBack(const sensor_msgs::Image::ConstPtr& image) {
 for (const auto & pixel_value : image->data) {
     if (pixel_value != 0) {
        pixel_counter_++;
     } 
  }
  if (pixel_counter_ >= PIXEL_REQ) {
    found_bag_ = true;
  } else {
    found_bag_ = false;
  }
  pixel_counter_ = 0;
}

void
DetectBag::halt()
{
  ROS_INFO("DetectBag halt");
}

BT::NodeStatus
DetectBag::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Bag");
  }

  if (found_bag_) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace carry_my_luggage