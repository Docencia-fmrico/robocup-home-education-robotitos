#include <carry_my_luggage/DetectBag.h>
#include <carry_my_luggage/CMLDialog.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

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
  sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &DetectPerson::DetectBagCallBack,this);
}


void
DetectBag::DetectBagCallBack(const sensor_msgs::Image::ConstPtr& image) {
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      if (!found_person_){
        found_person_ = true;
        px_init = (box.xmax + box.xmin) / 2;
        py_init = (box.ymax + box.ymin) / 2;
      }
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;
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
  fowarder.step();
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