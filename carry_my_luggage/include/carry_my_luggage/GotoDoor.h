#ifndef CARRY_MY_LUGGAGE_GOTODOOR_H
#define CARRY_MY_LUGGAGE_GOTODOOR_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

namespace carry_my_luggage
{

class GotoDoor : public BT::ActionNodeBase
{
  public:
    explicit GotoDoor(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();
    void GotoDoorCallBack(const sensor_msgs::Image::ConstPtr& image);

     static BT::PortsList providedPorts()
    {
        return{};
    }

  private:
    ros::NodeHandle n_;
};

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_GOTODOOR_H