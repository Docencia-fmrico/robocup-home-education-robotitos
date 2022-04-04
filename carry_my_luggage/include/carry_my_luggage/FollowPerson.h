#ifndef CARRY_MY_LUGGAGE_FOLLOWPERSON_H
#define CARRY_MY_LUGGAGE_FOLLOWPERSON_H

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
#include "carry_my_luggage/PID.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

class FollowPerson : public BT::ActionNodeBase
{
  public:
    explicit FollowPerson(const std::string& name);
    void halt();
    BT::NodeStatus tick();
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    void CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter);
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser);
    
  private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

    bool found_person_;
    ros::Subscriber sub_counter_;
    ros::Subscriber sub_laser_;
    bool obstacle_detected_;
    float dist;
    int px_min, px_max;
    int py, px;
    PID pid_foward = PID(1.4, 7, 0.0, 0.2); // PID solo para ir hacia delante
    PID pid_turn_right = PID(440, 640, 0.0, 0.4); // PID para girar a la derecha
    PID pid_turn_left = PID(0, 200, 0.0, 0.4); // PID para girar a la izquierda

};

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_FOLLOWPERSON_H