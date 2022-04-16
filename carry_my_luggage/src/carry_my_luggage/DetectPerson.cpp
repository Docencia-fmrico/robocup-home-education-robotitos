#include "carry_my_luggage/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {}), sync_bbx(MySyncPolicy_bbx(10),image_depth_sub, bbx_sub), obstacle_detected_(false),
  objectFrameId_("/object/0"), workingFrameId_("/base_footprint"), cameraTopicId_("/cloud_filtered/0"), found_person_(false)
{ 
  image_depth_sub.subscribe(n_, "/camera/depth/image_raw", 1);
  bbx_sub.subscribe(n_, "/darknet_ros/bounding_boxes", 1);
  sync_bbx.registerCallback(boost::bind(&DetectPerson::callback_bbx, this,  _1, _2));

  n_.param("object_id", objectFrameId_, objectFrameId_);
  n_.param("cloud_id", cameraTopicId_, cameraTopicId_);
  pointCloudSub_ = n_.subscribe(cameraTopicId_,5,&DetectPerson::cloudCB,this);
}

void DetectPerson::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  ROS_INFO(" callback person tf");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_in, *pcrgb);
  int pixel =py*image_width +px;
  auto point = pcrgb->at(pixel);
  
  tf::StampedTransform transform;
  if (!std::isnan(point.x) && !std::isnan(point.y))
  {
    transform.setOrigin(tf::Vector3(point.x, point.y, 0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    std::cout << "point.x: " << point.x << " point.y: " << point.y << std::endl;
    std::cout << "px: " << px << " py: " << py << std::endl;

    transform.stamp_ = ros::Time::now();
    transform.frame_id_ = workingFrameId_;
    transform.child_frame_id_ = objectFrameId_;

    try
    {
      tfBroadcaster_.sendTransform(transform);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }
  }
}

void
DetectPerson::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
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
  image_width = image->width;
  px_max = image->width;
  px_min = 0;
  found_person_ = false;

  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class == "person") {
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;
      found_person_ = true; //estaba aqui en falso y no se como funcionaba
      dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
    }
  }
}

void
DetectPerson::halt()
{
  ROS_INFO("DetectPerson halt");
}

BT::NodeStatus
DetectPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }


  return BT::NodeStatus::SUCCESS;
  /*
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
  */
}

}  // namespace carry_my_luggage