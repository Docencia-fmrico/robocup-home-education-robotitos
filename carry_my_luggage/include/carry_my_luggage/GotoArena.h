#ifndef CARRY_MY_LUGGAGE_GOTOARENA_H
#define CARRY_MY_LUGGAGE_GOTOARENA_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"

#define SECURITY_DISTANCE 0.45
#define CENTER_LASER 540

namespace carry_my_luggage
{

class GotoArena : public BTNavAction
{
  public:
    explicit GotoArena(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

     static BT::PortsList providedPorts()
    {
        return{};
    }

  private:
    ros::NodeHandle n_;
    int counter_;
};

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_GOTOARENA_H 