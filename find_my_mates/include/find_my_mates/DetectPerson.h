#ifndef FIND_MY_MATES_DETECTPERSON_H
#define FIND_MY_MATES_DETECTPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include "ros/ros.h"
#include <string>

namespace find_my_mates
{

class DetectPerson : public BT::ConditionNode
{
  public:
    explicit DetectPerson(const std::string& name, const BT::NodeConfiguration & config);

    BT::NodeStatus tick();
    void CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter);
    void DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    static BT::PortsList providedPorts() {
      return {};
    }
    
  private:
    ros::NodeHandle n_;
    
    bool found_person_;
    ros::Subscriber sub_counter_;
    ros::Subscriber sub_darknet_;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_DETECTPERSON_H