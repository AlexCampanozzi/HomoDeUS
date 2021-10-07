#include <point_cloud_perception/proc_point_cloud_perception.h>

CloudObjectFinder::CloudObjectFinder(ros::NodeHandle& nh): _nh(nh)
{
    // TODO:  put real topic & type where face detections will be here
    _detection_sub = _nh.subscribe("/bounding_boxes", 5, &CloudObjectFinder::detectionCallback, this);
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);
    _pub = _nh.advertise<geometry_msgs::PoseStamped>("/pick_position", 5);

    noplane_pub = _nh.advertise<sensor_msgs::PointCloud2>("/noplane_cloud", 5);
}

void CloudObjectFinder::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    bool found_object = false;
    for (auto box : msg->bounding_boxes)
    {
        if (box.Class == desired_object_type)
        {
            found_object = true;
            latest_detection = box;
        }
    }
    if (found_object)
    {
        // Stop taking detections into account while we process this one
        _detection_sub.shutdown();
        _cloud_sub = _nh.subscribe("/xtion/depth_registered/points", 5, &CloudObjectFinder::cloudCallback, this);
    }
}

void CloudObjectFinder::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, scene_cloud);
    // scene_cloud = *msg;

    std::vector<int> dummyIndices; //Just so we can call funcs that output indices;
    pcl::removeNaNFromPointCloud(scene_cloud, dummyIndices);

    // TODO: passtrhrough here depending on the position of the object detected in the image (may need head angle as input to do something clean?)
    // Put through passthrough filter to conserve only the points in the general region where we expect the target to be
    auto detectedX = (latest_detection.xmax + latest_detection.xmin)/2;
    auto detectedY = (latest_detection.ymax + latest_detection.ymin)/2;
    // auto detectedZ = latest_detection.pose.position.z;

    // Define a box around the detection in which we keep information 
    float box_half_width = 0.5;
    float box_half_height = 0.5;

    PointCloud point_cloud_xfiltered;
    PointCloud point_cloud_xyfiltered;
    PointCloud point_cloud_filtered;

    pcl::PassThrough<pcl::PointXYZ> passx;
    passx.setInputCloud(scene_cloud.makeShared());
    passx.setFilterFieldName("x");
    passx.setFilterLimits(detectedX-box_half_width, detectedX+box_half_width);
    // passx.setFilterLimitsNegative (true);
    passx.filter(point_cloud_xfiltered);

    pcl::PassThrough<pcl::PointXYZ> passy;
    passy.setInputCloud(point_cloud_xfiltered.makeShared());
    passy.setFilterFieldName("y");
    passy.setFilterLimits(detectedY-box_half_height, detectedY+box_half_height);
    //pass.setFilterLimitsNegative (true);
    // passy.filter(point_cloud_xyfiltered);
    passy.filter(point_cloud_filtered);

    // Detection will be form a 2D image: no info on depth, and therefore we keep all of it
    // pcl::PassThrough<pcl::PointXYZ> passz;
    // passz.setInputCloud(point_cloud_xyfiltered.makeShared());
    // passz.setFilterFieldName("z");
    // passz.setFilterLimits(-2, 3);
    // //pass.setFilterLimitsNegative (true);
    // passz.filter(point_cloud_filtered);


    // Put the worked cloud in the robot's frame so operation directions make sense to us
    pcl_ros::transformPointCloud("base_link", ros::Time::now(), scene_cloud, scene_cloud.header.frame_id, scene_cloud, tfBuffer);


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

    seg.setInputCloud(point_cloud_filtered.makeShared());
    seg.segment(*inliers, *coefficients);

    PointCloud planeCloud;

    // Make new cloud with inliers only
    pcl::copyPointCloud(point_cloud_filtered, *inliers, planeCloud);

    // Make new cloud with outliers: without the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr noPlane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // TODO: use filtered cloud
    extract.setInputCloud(point_cloud_filtered.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*noPlane);
    pcl::io::savePCDFile("noplane.pcd", *noPlane);

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
