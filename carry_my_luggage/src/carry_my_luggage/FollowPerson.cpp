#include "carry_my_luggage/FollowPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include "sensor_msgs/LaserScan.h"
#include "carry_my_luggage/PID.h"

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

FollowPerson::FollowPerson(const std::string& name)
: BT::ActionNodeBase(name, {}), sync_bbx(MySyncPolicy_bbx(10),image_depth_sub, bbx_sub), obstacle_detected_(false)
{ found_person_ == false;
  image_depth_sub.subscribe(n_, "/camera/depth/image_raw", 1);
  bbx_sub.subscribe(n_, "/darknet_ros/bounding_boxes", 1);

  sub_counter_ = n_.subscribe("/darknet_ros/found_object", 1, &FollowPerson::CounterCallBack,this);
  sync_bbx.registerCallback(boost::bind(&FollowPerson::callback_bbx, this,  _1, _2));
  //sub_laser_ = n_.subscribe("/scan",1,&FollowPerson::laserCallBack,this);
}

void
FollowPerson::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
  ROS_INFO(" callback detectperson dist");
  cv_bridge::CvImagePtr img_ptr_depth;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  px_max = image->width;
  px_min = 0;
  found_person_ = false;
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class == "person") {
      px = (box.xmax + box.xmin) / 2;
      int py = (box.ymax + box.ymin) / 2;
      found_person_ = true; //estaba aqui en falso y no se como funcionaba
      dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
    }
  }
}

void
FollowPerson::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    int min_izq = laser->range_max*0.9;
    int max_dcha = laser->range_max*0.1;

    int dist_min = 0.5;

    for (int i = laser->range_min; i <= max_dcha; i++) {
        if (laser->ranges[i] <= dist_min && laser->ranges[i] != 0.0){
            obstacle_detected_=true;
        }
    }
    for (int i = laser->range_max; i >= min_izq; i--) {
        if (laser->ranges[i] <= dist_min && laser->ranges[i] != 0.0){
            obstacle_detected_=true;
        }
    }
}

void
FollowPerson::CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter) {
  ROS_INFO(" callback counter");

  if (counter->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;
  }
}

void
FollowPerson::halt()
{
  ROS_INFO("FollowPerson halt");
}

BT::NodeStatus
FollowPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }

  double foward_velocity = 0.0; //pid_foward.get_output(dist);
  double turn_right_velocity = 0.0; //pid_turn_right.get_output(px);
  double turn_left_velocity = 0.0; //pid_turn_left.get_output(px);

  if (dist <= 1.0) {       //si esta muy cerca deja de detectar a la persona entonces si la ultima dist
    found_person_ == true; // es menor que 1.0 significa que lo tiene delante
  }

  if (found_person_ == true) {
    std::cerr << "dist:" << dist << std::endl;

    if (dist >= 1.4) {
      setOutput("foward_velocity", std::to_string(foward_velocity));
    } else if (dist <= 1.0) {
      setOutput("foward_velocity", "-0.1" );
    } else {
      setOutput("foward_velocity", "0.0" );
    }

    //comprobamos si debemos girar un poco o no dependiendo de donde se encuentre la persona
    if (px >= 440) {
      setOutput("turn_velocity", std::to_string(-turn_right_velocity));
    } else if (px <= 200) {
      setOutput("turn_velocity", std::to_string(turn_left_velocity));
    } else {
      setOutput("turn_velocity", "0.0");
    }
    
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("foward_velocity", "0.0" );
    setOutput("turn_velocity", "0.0" );
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace carry_my_luggage