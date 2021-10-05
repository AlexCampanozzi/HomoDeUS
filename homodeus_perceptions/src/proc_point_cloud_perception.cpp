#include <point_cloud_perception/proc_point_cloud_perception.h>

CloudObjectFinder::CloudObjectFinder(ros::NodeHandle& nh): _nh(nh)
{
    // TODO:  put real topic & type where face detections will be here
    _detection_sub = _nh.subscribe("/object_detections", 5, &CloudObjectFinder::detectionCallback, this);
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);
    _pub = nh.advertise<geometry_msgs::PoseStamped>("/pick_position", 5);
}

void CloudObjectFinder::detectionCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    latest_detection = *msg;
    // Stop taking detections into account while we process this one
    _detection_sub.shutdown();
    _cloud_sub = _nh.subscribe("/xtion/depth_registered/points", 5, &CloudObjectFinder::cloudCallback, this);
}

void CloudObjectFinder::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, scene_cloud);
    // scene_cloud = *msg;

    std::vector<int> dummyIndices; //Just so we can call funcs that output indices;
    pcl::removeNaNFromPointCloud(scene_cloud, dummyIndices);

    // TODO: passtrhrough here depending on the position of the object detected in the image (may need head angle as input to do something clean?)
    // Put through passthrough filter to conserve only the points in the general region where we expect the target to be
    // PointCloud point_cloud_xfiltered;
    // PointCloud point_cloud_xyfiltered;
    // PointCloud point_cloud_filtered;

    // pcl::PassThrough<pcl::PointXYZ> passx;
    // passx.setInputCloud(scene_cloud.makeShared());
    // passx.setFilterFieldName("x");
    // passx.setFilterLimits(0.35, 3);
    // // passx.setFilterLimitsNegative (true);
    // passx.filter(point_cloud_xfiltered);

    // pcl::PassThrough<pcl::PointXYZ> passy;
    // passy.setInputCloud(point_cloud_xfiltered.makeShared());
    // passy.setFilterFieldName("y");
    // passy.setFilterLimits(-1, 1);
    // //pass.setFilterLimitsNegative (true);
    // passy.filter(point_cloud_xyfiltered);

    // pcl::PassThrough<pcl::PointXYZ> passz;
    // passz.setInputCloud(point_cloud_xyfiltered.makeShared());
    // passz.setFilterFieldName("z");
    // passz.setFilterLimits(-2, 3);
    // //pass.setFilterLimitsNegative (true);
    // passz.filter(point_cloud_filtered);


    // Put the worked cloud in the robot's frame so operation directions make sense to us
    pcl_ros::transformPointCloud("base_link", ros::Time::now(), scene_cloud, latest_detection.header.frame_id, scene_cloud, tfBuffer);


    // Extract table plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    // Table should be perpendicular to the z axis (in the x-y plane)
    Eigen::Vector3f xAxis = Eigen::Vector3f(0,0,1);
    seg.setAxis(xAxis);
    seg.setEpsAngle(0.2);

    // TODO: use filtered cloud
    seg.setInputCloud(scene_cloud.makeShared());
    // seg.setInputCloud(point_cloud_filtered);
    seg.segment(*inliers, *coefficients);

    PointCloud planeCloud;

    // Make new cloud with inliers only
    // TODO: use filtered cloud
    pcl::copyPointCloud(scene_cloud, *inliers, planeCloud);
    // pcl::copyPointCloud(*cloud_filtered, *inliers, planeCloud);

    // Make new cloud with outliers: without the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr noPlane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // TODO: use filtered cloud
    extract.setInputCloud(scene_cloud.makeShared());
    // extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*noPlane);

    // TODO?: segment and focus on largest cloud in case there is still junk in the ROI

    // Find center of the object by averaging remaing points

    pcl::PointXYZ average_point;
    int numpoints = 0;
    float avgX = 0;
    float avgY = 0;
    float avgZ = 0;

    for (auto point : noPlane->points)
        {
            avgX += point.x;
            avgY += point.y;
            avgZ += point.z;
            numpoints += 1;
        }

    average_point = pcl::PointXYZ(avgX, avgY, avgZ);

    // TODO: find a way to decide orientation of pick point, either from shape, object identity, or both

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = average_point.x;
    goal_pose.pose.position.y = average_point.y;
    goal_pose.pose.position.z = average_point.z;

    // TODO: insert orientation here

    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = ros::Time::now();

}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"homodeus_proc_point_cloud_perception_node");

  ros::NodeHandle nh;

  double frequency = 5;

  ROS_INFO("Creating cloud object finder");

  CloudObjectFinder finder(nh);
  ROS_INFO("Spinning to serve callbacks ...");

  ros::Rate rate(frequency);
  while ( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
