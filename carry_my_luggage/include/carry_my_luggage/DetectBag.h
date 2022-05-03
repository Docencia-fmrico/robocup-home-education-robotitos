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
    bool found_bag_;
    int pixel_counter_;
    ros::NodeHandle n_;
    ros::Subscriber sub_hsv_;
};

/*class Dialog : public DialogInterface
{
  public: 
    Dialog();
    std::string getContext(dialogflow_ros_msgs::DialogflowResult result);
    std::string getIntent(dialogflow_ros_msgs::DialogflowResult result);
    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void detectBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void chooseBagIntentCB(dialogflow_ros_msgs::DialogflowResult result);

  private:
    int state_;
    std::string context_;
    std::string intent_;
    static const int IDLE = 0;
    static const int SPEAK = 1;
    static const int LISTEN = 2;
    ros::Time speak_ts_;

};*/

}  // namespace carry_my_luggage

#endif  // CARRY_MY_LUGGAGE_DETECTBAG_H