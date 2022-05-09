#include "carry_my_luggage/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
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

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "ros/ros.h"
#include <string>

namespace carry_my_luggage
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {}), sync_bbx(MySyncPolicy_bbx(10),image_depth_sub, bbx_sub), obstacle_detected_(false),
  objectFrameId_("/object/0"), workingFrameId_("/base_footprint"), cameraTopicId_("/camera/depth_registered/points"), found_person_(false) // depth_registered
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
  sensor_msgs::PointCloud2 cloud;

  try
  {
    pcl_ros::transformPointCloud(workingFrameId_, *cloud_in, cloud, tfListener_);
  }
  catch(tf::TransformException & ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(cloud, *pcrgb);

  int pixel =py*image_width +px;
  auto point = pcrgb->at(pixel);
  
  tf2::Stamped<tf2::Transform> transform;
  if (found_person_) {
    if (!std::isnan(point.x) && !std::isnan(point.y))
    {
      transform.setOrigin(tf2::Vector3(point.x, point.y, 0));
      transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      transform.stamp_ = ros::Time::now();
      transform.frame_id_ = workingFrameId_;
      geometry_msgs::TransformStamped object_msg = tf2::toMsg(transform);
      object_msg.child_frame_id = objectFrameId_;

      try
      {
        tfBroadcaster_.sendTransform(object_msg);
      }
      catch(tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }
    }
  }
}

void
DetectPerson::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
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

  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class == "person") {
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;
      found_person_ = true;
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
  found_person_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace carry_my_luggage