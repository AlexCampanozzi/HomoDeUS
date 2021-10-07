#ifndef POINTCLOUDPERCEPTION_H
#define POINTCLOUDPERCEPTION_H


// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pcl headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

/* CloudObjectFinder
Description:    This class analyses point clouds to get an object pose for picking
Attributes:     _nh (type, ros::NodeHandle):
                    Necessary for the proper handling of a ROS node

                _pub (type, ros::Publisher):
                    The topic to publish the coordinates on
                
*/
class CloudObjectFinder
{
public:
    CloudObjectFinder(ros::NodeHandle& nh);
    // CloudObjectFinder();
    // virtual ~CloudObjectFinder();

protected:

  ros::NodeHandle _nh;
  ros::Publisher _pub;
  // for debug
  ros::Publisher noplane_pub;
  ros::Subscriber _detection_sub;
  ros::Subscriber _cloud_sub;

  geometry_msgs::PoseStamped latest_detection;
  PointCloud scene_cloud;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListenerPtr;
  
  void detectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

};

#endif