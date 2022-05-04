#ifndef CARRY_MY_LUGGAGE_GOTOPERSON_H
#define CARRY_MY_LUGGAGE_GOTOPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "carry_my_luggage/BTNavAction.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

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

namespace carry_my_luggage
{

class GotoPerson : public BTNavAction
{
  public:
    explicit GotoPerson(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;
    void DirectionCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position);
    void GotoPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    ros::NodeHandle n_;
    ros::Subscriber direction_;
    ros::Subscriber sub_darknet_ ;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    move_base_msgs::MoveBaseGoal directions;

    geometry_msgs::TransformStamped bf2odom_msg;
    geometry_msgs::TransformStamped odom2obj_msg;
    geometry_msgs::TransformStamped map2bf_msg;
    tf2::Stamped<tf2::Transform> odom2obj;
    tf2::Stamped<tf2::Transform> bf2odom;
    tf2::Stamped<tf2::Transform> map2bf;
    tf2::Transform bf2obj;
    std::string error;

    int counter_, px;
};

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_GOTOPERSON_H 