#ifndef CARRY_MY_LUGGAGE_DETECTBAG_H
#define CARRY_MY_LUGGAGE_DETECTBAG_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <carry_my_luggage/CMLDialog.h>

#include <string>

namespace carry_my_luggage
{
class DetectBag : public BT::ActionNodeBase
{
  public:
    explicit DetectBag(const std::string& name, const BT::NodeConfiguration& config);
    void step();

    BT::NodeStatus tick();
    void halt();
    void DetectBagCallBack(const sensor_msgs::Image::ConstPtr& image);


    static BT::PortsList providedPorts()
    {
      return{};
    }

  private:
    bool found_person_;
    int pixel_counter_;
    int px_init, py_init;
    int px, py;
    ros::NodeHandle n_;
    ros::Subscriber sub_hsv_;
    carry_my_luggage::Dialog fowarder;
};
}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_DETECTBAG_H