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
    double person_1[4] = {-5.05, 2.55, 0.10, 1.0};
    double person_2[4] = {-4.65, -3.85, 0.05, 1.0};
    double person_3[4] = {-4.65, -4.80, 0.50, 1.0};
    double person_4[4] = {-3.65, -4.35, -0.65, 1.0};
    double person_5[4] = {-0.55, -4.15, 0.70, 1.0};
    double person_6[4] = {-2.45, -1.95, -0.99, 1.0};
    double person[6][4] = {{-5.05, 2.55, 0.10, 1.0}, {-4.65, -3.85, 0.05, 1.0}, {-4.65, -4.80, 0.50, 1.0}, {-3.65, -4.35, -0.65, 1.0}, {-0.55, -4.15, 0.70, 1.0}, {-2.45, -1.95, -0.99, 1.0}};
    bool goal_sent;
    ros::NodeHandle n_;

    ros::Subscriber direction_;
    move_base_msgs::MoveBaseGoal directions;
    int count;

};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_GOTOPERSON_H