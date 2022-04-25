#ifndef FIND_MY_MATES_DETECTPERSON_H
#define FIND_MY_MATES_DETECTPERSON_H

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

namespace find_my_mates
{

class DetectPerson : public BT::ActionNodeBase
{
  public:
    explicit DetectPerson(const std::string& name, const BT::NodeConfiguration & config);

    void halt();
    BT::NodeStatus tick();
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
   
    static BT::PortsList providedPorts() {
      return {};
    }
    
  private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

    tf::MessageFilter<sensor_msgs::PointCloud2> *tfPointCloudSub_;
    
    bool found_person_;
    ros::Subscriber sub_counter_;
    ros::Subscriber sub_laser_;
    ros::Subscriber pointCloudSub_;
    tf::TransformBroadcaster tfBroadcaster_;

    std::string objectFrameId_;
    std::string workingFrameId_;
    std::string cameraTopicId_;
    
    bool obstacle_detected_;
    float dist;
    int px_min, px_max;
    int py, px, image_width;
};

}  // namespace find_my_mates

#endif  // FIND_MY_MATES_DETECTPERSON_H