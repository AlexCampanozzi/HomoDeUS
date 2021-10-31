#include <point_cloud_perception/proc_point_cloud_perception.h>

CloudObjectFinder::CloudObjectFinder(ros::NodeHandle& nh): _nh(nh)
{
    _detection_sub = _nh.subscribe("/darknet_ros/bounding_boxes", 5, &CloudObjectFinder::detectionCallback, this);
    image_info_sub  = _nh.subscribe("/xtion/rgb/camera_info", 5, &CloudObjectFinder::imageInfoCallback, this);
    desired_object_sub  = _nh.subscribe("/desired_object", 5, &CloudObjectFinder::desiredObjectCallback, this);
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);
    _pub = _nh.advertise<geometry_msgs::PoseStamped>("/pick_position", 5);

    noplane_pub = _nh.advertise<sensor_msgs::PointCloud2>("/noplane_cloud", 5);
    filtered_pub = _nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 5);
    pick_point_pub = _nh.advertise<geometry_msgs::PoseStamped>("/pick_point", 5);


}

void CloudObjectFinder::imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& info)
{
    camera_info = *info;
    camera_model.fromCameraInfo(camera_info);
    image_info_sub.shutdown();
    got_cam_info = true;
    ROS_INFO("Got camera info");
    std::cout << "Info: " << camera_info << std::endl;
}

void CloudObjectFinder::desiredObjectCallback(const std_msgs::StringConstPtr& type)
{
    desired_object_type = type->data;
    ROS_INFO("Now looking for %s", desired_object_type.c_str());
}

void CloudObjectFinder::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    ROS_INFO("In box callback");
    bool found_object = false;
    for (auto box : msg->bounding_boxes)
    {
        if (box.Class == desired_object_type)
        {
            ROS_INFO("Object of desired class found");
            found_object = true;
            latest_detection = box;
            std::cout << "Detection: " << latest_detection << std::endl;
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
    if (!got_cam_info)
    {
        // then we cant transform pixels to coords yet, retry later
        ROS_INFO("Did not have cam_info when attempting to process cloud, will try again on next CB");
        return;
    }

    // Unsub so we dont keep analysing the same thing w/o a new detection
    desired_object_type = "";
    _cloud_sub.shutdown();
    pcl::fromROSMsg(*msg, scene_cloud);
    // scene_cloud = *msg;
    ROS_INFO("In cloud callback for %lu points", scene_cloud.size());
    ROS_INFO("Is cloud dense: %d", scene_cloud.is_dense);

    std::vector<int> dummyIndices; //Just so we can call funcs that output indices;
    pcl::removeNaNFromPointCloud(scene_cloud, scene_cloud, dummyIndices);

    ROS_INFO("Removed NAN");
    ROS_INFO("Cloud now has %lu points", scene_cloud.size());
    ROS_INFO("Cloud is now dense: %d", scene_cloud.is_dense);

    // Put through passthrough filter to conserve only the points in the general region where we expect the target to be
    auto detectedXpix = (latest_detection.xmax + latest_detection.xmin)/2;
    auto detectedYpix = (latest_detection.ymax + latest_detection.ymin)/2;
    std::cout << "Object center in px: (" << detectedXpix << ", " << detectedYpix << ")" << std::endl;
    auto detected_coordinates = camera_model.projectPixelTo3dRay(cv::Point2d(detectedXpix, detectedYpix));
    auto detectedX = detected_coordinates.x;
    auto detectedY = detected_coordinates.y;

    std::cout << "Object center in camera frame: (" << detected_coordinates << ")" << std::endl;

    // auto detectedZ = latest_detection.pose.position.z;

    // Define a box around the detection in which we keep information 
    // TODO: passthrough here depending on the dimensions instead of center position
    float box_half_width = 0.1;
    float box_half_height = 0.2;

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

    ROS_INFO("Filtered with passthrough");

    pcl::toROSMsg(point_cloud_filtered, _filtered_cloud);
    _filtered_cloud.header.frame_id = "xtion_rgb_optical_frame";
    // ros::Timer filtered_timer = _nh.createTimer(ros::Duration(2), &CloudObjectFinder::filteredTimerCallback, this);
    // filtered_timer.start();
    filtered_pub.publish(_filtered_cloud);
    ROS_INFO("Cloud now has %lu points", point_cloud_filtered.size());


    // Put the worked cloud in the robot's frame so operation directions make sense to us
    auto curTime = ros::Time::now();
    auto curTransform = tfBuffer.lookupTransform(ref_frame, scene_cloud.header.frame_id, curTime, ros::Duration(10));
    // pcl_ros::transformPointCloud(ref_frame, curTime, scene_cloud, scene_cloud.header.frame_id, scene_cloud, tfBuffer);
    pcl_ros::transformPointCloud(ref_frame, curTime, point_cloud_filtered, scene_cloud.header.frame_id, point_cloud_filtered, tfBuffer);


    // pcl::toROSMsg(scene_cloud, _filtered_cloud);
    // _filtered_cloud.header.frame_id = ref_frame;
    // filtered_pub.publish(_filtered_cloud);

    ROS_INFO("Moved to base_link frame");

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
    seg.setDistanceThreshold(0.02);
    // Table should be perpendicular to the z axis (in the x-y plane)
    Eigen::Vector3f zAxis = Eigen::Vector3f(0,0,1);
    seg.setAxis(zAxis);
    seg.setEpsAngle(0.2);

    seg.setInputCloud(point_cloud_filtered.makeShared());
    seg.segment(*inliers, *coefficients);

    ROS_INFO("Segmented plane");

    PointCloud planeCloud;

    // Make new cloud with inliers only
    pcl::copyPointCloud(point_cloud_filtered, *inliers, planeCloud);

    // Old: Make new cloud with outliers: without the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr noPlane(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("noplane initial frame: %s", noPlane->header.frame_id.c_str());
    noPlane->header.frame_id = ref_frame;
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(point_cloud_filtered.makeShared());
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*noPlane);

    // New: Passthrough in z to remove anything below top of table

    float maxz = -999;
    for (auto point : planeCloud)
    {
        if(point.z > maxz)
        {
            maxz = point.z;
        }
    }

    pcl::PassThrough<pcl::PointXYZ> passz;
    passz.setInputCloud(point_cloud_filtered.makeShared());
    passz.setFilterFieldName("z");
    passz.setFilterLimits(-999, maxz);
    passz.setFilterLimitsNegative (true);
    passz.filter(*noPlane);

    pcl::io::savePCDFile("noplane.pcd", *noPlane);
    pcl::toROSMsg(*noPlane, _noplane_cloud);
    // ros::Timer noplane_timer = _nh.createTimer(ros::Duration(2), &CloudObjectFinder::noplaneTimerCallback, this);
    // noplane_timer.start();
    ROS_INFO("noplane final frame: %s", _noplane_cloud.header.frame_id.c_str());
    _noplane_cloud.header.frame_id = ref_frame;
    noplane_pub.publish(_noplane_cloud);

    ROS_INFO("Removed plane");

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

    average_point = pcl::PointXYZ(avgX/numpoints, avgY/numpoints, avgZ/numpoints);

    // TODO: find a way to decide orientation of pick point, either from shape, object identity, or both

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = average_point.x;
    goal_pose.pose.position.y = average_point.y;
    goal_pose.pose.position.z = average_point.z;

    // TODO: insert orientation here

    goal_pose.header.frame_id = ref_frame;
    goal_pose.header.stamp = ros::Time::now();
    _pick_pose = goal_pose;
    ROS_INFO("Pick pose found");
    std::cout << "pose: " << std::endl << _pick_pose << std::endl;
    // ros::Timer pickpoint_timer = _nh.createTimer(ros::Duration(2), &CloudObjectFinder::pickpointTimerCallback, this);
    // pickpoint_timer.start();
    pick_point_pub.publish(_pick_pose);
    got_pick_pose = true;

    // Feed pos directly into arm controller for initial testing approach
    // if(arm_interface.moveToCartesian(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, 0, 0, 0))
    // {
    //     ROS_INFO("Succesfully moved to pose");
    // }
    // else
    // {
    //     ROS_INFO("Unable to move ");
    // }
    
}

void CloudObjectFinder::noplaneTimerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Publishing noplane");
    noplane_pub.publish(_noplane_cloud);
}
void CloudObjectFinder::filteredTimerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Publishing filtered");
    filtered_pub.publish(_filtered_cloud);
}
void CloudObjectFinder::pickpointTimerCallback(const ros::TimerEvent&)
{
    ROS_INFO("Publishing pose");
    pick_point_pub.publish(_pick_pose);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"homodeus_proc_point_cloud_perception_node");

    ros::NodeHandle nh;

    double frequency = 5;

    ROS_INFO("Creating cloud object finder");

    CloudObjectFinder finder(nh);
    // ArmInterface arm_interface(ref_frame);
    ROS_INFO("Spinning to serve callbacks ...");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ros::Rate rate(frequency);
    // while ( ros::ok() && !finder.pick_pose_found() )
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // if(ros::ok())
    // {
    //     auto position = finder.get_pick_pose().pose.position;
    //     ROS_INFO("arm_interface will attempt to move the arm in cartesian space.");
    //     auto success = arm_interface.moveToCartesian(position.x-0.2, position.y, position.z, 0, 0, 0);
    //     if (success)
    //         ROS_INFO("arm_interface_node: succeeded!");

    //     else
    //         ROS_INFO("arm_interface_node: failed!");


    //     ros::waitForShutdown();   
    // }

    ros::waitForShutdown();
     

    return 0;
}
