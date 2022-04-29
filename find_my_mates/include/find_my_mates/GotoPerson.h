#ifndef FIND_MY_MATES_GOTOPERSON_H
#define FIND_MY_MATES_GOTOPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "find_my_mates/BTNavAction.h"

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/message_filter.h"

#include "ros/ros.h"
#include <string>

namespace find_my_mates
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
    void DirectionCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position);

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    ros::NodeHandle n_;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    ros::Subscriber direction_;
    move_base_msgs::MoveBaseGoal directions;

    geometry_msgs::TransformStamped bf2person_msg;
    tf2::Stamped<tf2::Transform> bf2person;
    std::string error;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_GOTOPERSON_H