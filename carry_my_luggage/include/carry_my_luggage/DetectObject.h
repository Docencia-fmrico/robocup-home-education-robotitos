#ifndef VISUAL_BEHAVIOR_DETECTOBJECT_H
#define VISUAL_BEHAVIOR_DETECTOBJECT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
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
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser);
   
    static BT::PortsList providedPorts()
    {
        return {};
    }
    
  private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

    ros::Subscriber sub_laser_;
    bool obstacle_detected_;
    float dist;
    int px_min, px_max;
    int py, px;
};

}  // namespace carry_my_luggage

#endif  // VISUAL_BEHAVIOR_DETECTOBJECT_H