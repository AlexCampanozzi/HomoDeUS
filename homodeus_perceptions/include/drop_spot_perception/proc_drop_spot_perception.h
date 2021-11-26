#ifndef DROPSPOTPERCEPTION_H
#define DROPSPOTPERCEPTION_H


// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_geometry/pinhole_camera_model.h>

// pcl headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

const std::string ref_frame = "base_link";

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

struct extremities {
  float min;
  float max;
  extremities(float min, float max) : min{min}, max{max} {}
  bool isWithin(float val)
  {
    return (val > min && val < max);
  }
  bool isWithinWtihTol(float val, float tol)
  {
    return (val > min-tol && val < max+tol);
  }
};

/* DropSpotFinder
Description:    This class analyses point clouds to get an object pose for droping
Attributes:     _nh (type, ros::NodeHandle):
                    Necessary for the proper handling of a ROS node

                _pub (type, ros::Publisher):
                    The topic to publish the coordinates on
                
*/
class DropSpotFinder
{
public:
    DropSpotFinder(ros::NodeHandle& nh);
    // DropSpotFinder();
    // virtual ~DropSpotFinder();
    inline const bool drop_pose_found(){return got_drop_pose;}
    inline geometry_msgs::PoseStamped get_drop_pose(){return _drop_pose;}

protected:

  ros::NodeHandle _nh;

  // for debug
  sensor_msgs::PointCloud2 filtered_cloud;
  geometry_msgs::PoseStamped _drop_pose;
  geometry_msgs::TransformStamped grip_to_wrist_tf;
  ros::Publisher drop_point_pub;
  ros::Publisher plane_cloud_pub;
  sensor_msgs::PointCloud2 plane_cloud;

  ros::Subscriber _cloud_sub;
  ros::Subscriber image_info_sub;
  ros::Subscriber desired_object_sub;

  // Temp, for use outside HHBA
  // ros::Subscriber trigger_sub;
  // void triggerCallback(const std_msgs::EmptyConstPtr& nothing);

  ros::Subscriber object_height_sub;
  // height at which we picked the objet, default value to avoid collisions
  float object_height = 0.15;

  darknet_ros_msgs::BoundingBox latest_detection;
  PointCloud scene_cloud;

  sensor_msgs::CameraInfo camera_info;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListenerPtr;

  bool got_cam_info = false;

  bool got_drop_pose = false;

  image_geometry::PinholeCameraModel camera_model;
  
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& info);
  void objectHeightCallback(const std_msgs::Float32ConstPtr& height);
};

#endif