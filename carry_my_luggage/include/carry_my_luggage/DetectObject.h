#ifndef VISUAL_BEHAVIOR_DETECTOBJECT_H
#define VISUAL_BEHAVIOR_DETECTOBJECT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

class DetectObject : public BT::ConditionNode
{
  public:
    explicit DetectObject(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser);
   
    static BT::PortsList providedPorts()
    {
        return {};
    }
    
  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_laser_;
    ros::Publisher pub_vel_;
    bool obstacle_detected_;
    int obstacle_state_, state_;

    static const int GOING_FORWARD = 0;
    static const int GOING_BACK = 1;
    static const int TURNING_LEFT = 2;
    static const int TURNING_RIGHT = 3;
    static const int LEFT_DETECTED = 0;
    static const int RIGHT_DETECTED = 1;
    static const int CENTER_DETECTED = 2;

    static constexpr double TURNING_TIME = 3.0;
    static constexpr double BACKING_TIME = 3.0;

    ros::Time detected_ts_;
    ros::Time turn_ts_;
};

}  // namespace carry_my_luggage

#endif  // VISUAL_BEHAVIOR_DETECTOBJECT_H