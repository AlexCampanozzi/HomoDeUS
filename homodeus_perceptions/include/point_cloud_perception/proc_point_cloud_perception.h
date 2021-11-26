#ifndef POINTCLOUDPERCEPTION_H
#define POINTCLOUDPERCEPTION_H


// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
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

const std::string ref_frame = "base_link";

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
    inline const bool pick_pose_found(){return got_pick_pose;}
    inline geometry_msgs::PoseStamped get_pick_pose(){return _pick_pose;}

protected:

  ros::NodeHandle _nh;

  // for debug
  sensor_msgs::PointCloud2 _filtered_cloud;
  sensor_msgs::PointCloud2 _noplane_cloud;
  geometry_msgs::PoseStamped _pick_pose;
  geometry_msgs::TransformStamped grip_to_wrist_tf;

  ros::Publisher filtered_pub;
  ros::Publisher noplane_pub;
  ros::Publisher pick_point_pub;
  ros::Publisher object_height_pub;

  ros::Subscriber _detection_sub;
  ros::Subscriber _cloud_sub;
  ros::Subscriber image_info_sub;
  ros::Subscriber desired_object_sub;

  darknet_ros_msgs::BoundingBox latest_detection;
  PointCloud scene_cloud;

  std::string desired_object_type;
  sensor_msgs::CameraInfo camera_info;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListenerPtr;

  bool got_cam_info = false;

  bool got_pick_pose = false;

  image_geometry::PinholeCameraModel camera_model;
  
  void detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& info);
  void desiredObjectCallback(const std_msgs::StringConstPtr& type);

  void resetSearch();

};

#endif