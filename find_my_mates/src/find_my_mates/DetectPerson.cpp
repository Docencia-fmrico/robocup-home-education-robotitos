#include "find_my_mates/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include "ros/ros.h"
#include <string>

namespace find_my_mates
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),found_person_(false)
{
  sub_counter_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPerson::CounterCallBack,this);
  sub_darknet_ = n_.subscribe("/darknet_ros/bounding_boxes", 1, &DetectPerson::DetectPersonCallBack,this);
}

void
DetectPerson::CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter) {
  ROS_INFO(" callback counter");

  if (counter->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;
  }
}

void
DetectPerson::DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
  ROS_INFO(" callback detectperson");
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      found_person_ = true;
    } else {
      found_person_ = false;
    }
  }
}

BT::NodeStatus
DetectPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person");
  }

  if (found_person_) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace find_my_mates