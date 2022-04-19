#ifndef CARRY_MY_LUGGAGE_GOTOPERSON_H
#define CARRY_MY_LUGGAGE_GOTOPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

class GotoPerson : public BTNavAction
{
  public:
    explicit GotoPerson(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    
};

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_GOTOPERSON_H