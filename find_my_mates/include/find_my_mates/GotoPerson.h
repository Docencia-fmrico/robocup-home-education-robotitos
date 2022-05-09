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
#include <darknet_ros_msgs/BoundingBoxes.h>

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
    void GotoPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    static BT::PortsList providedPorts() {
      return {};
    }
  private:
    double person[6][4] = {{3.326, 6.175, 1.586, 1.0}, {1.345, 6.966, 1.586, 1.0}, {1.074, 5.846, 2.88, 1.0}, {1.218, 4.163, -2.825, 1.0}, {1.65, 3.028, -2.169, 1.0}, {2.822, 2.582, -1.719, 1.0}};
    bool goal_sent;
    ros::NodeHandle n_;

    ros::Subscriber direction_;
    move_base_msgs::MoveBaseGoal directions;
    int count, counter_;
    ros::Subscriber sub_darknet_;
    bool found_person_;

};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_GOTOPERSON_H