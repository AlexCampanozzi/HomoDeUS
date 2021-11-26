#include <point_cloud_perception/proc_point_cloud_perception.h>

CloudObjectFinder::CloudObjectFinder(ros::NodeHandle& nh): _nh(nh)
{
    _detection_sub = _nh.subscribe("bounding_boxes", 5, &CloudObjectFinder::detectionCallback, this);
    image_info_sub  = _nh.subscribe("/xtion/rgb/camera_info", 5, &CloudObjectFinder::imageInfoCallback, this);
    desired_object_sub  = _nh.subscribe("/desired_object", 5, &CloudObjectFinder::desiredObjectCallback, this);
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);

    noplane_pub = _nh.advertise<sensor_msgs::PointCloud2>("/noplane_cloud", 5);
    filtered_pub = _nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 5);
    pick_point_pub = _nh.advertise<geometry_msgs::PoseStamped>("/pick_point", 5);
    object_height_pub = _nh.advertise<std_msgs::Float32>("/object_pick_height", 1);
    grip_to_wrist_tf =  tfBuffer.lookupTransform("gripper_grasping_frame", "arm_tool_link", ros::Time(0), ros::Duration(30));
    std::cout << grip_to_wrist_tf << std::endl;
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
    // ROS_INFO("In box callback");
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
        _cloud_sub = _nh.subscribe("points", 5, &CloudObjectFinder::cloudCallback, this);
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
    auto detected_coordinates = camera_model.projectPixelTo3dRay(camera_model.rectifyPoint(cv::Point2d(detectedXpix, detectedYpix)));
    auto detected_top_right = camera_model.projectPixelTo3dRay(camera_model.rectifyPoint(cv::Point2d(latest_detection.xmax, latest_detection.ymax)));
    auto detected_bottom_left = camera_model.projectPixelTo3dRay(camera_model.rectifyPoint(cv::Point2d(latest_detection.xmin, latest_detection.ymin)));
    auto detectedX = detected_coordinates.x;
    auto detectedY = detected_coordinates.y;
    auto detected_top  = detected_top_right.y;
    auto detected_bottom  = detected_bottom_left.y;
    auto detected_left  = detected_bottom_left.x;
    auto detected_right  = detected_top_right.x;

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
    filtered_pub.publish(_filtered_cloud);
    ROS_INFO("Cloud now has %lu points", point_cloud_filtered.size());


    // Put the worked cloud in the robot's frame so operation directions make sense to us
    auto curTime = ros::Time::now();
    auto curTransform = tfBuffer.lookupTransform(ref_frame, scene_cloud.header.frame_id, curTime, ros::Duration(10));
    pcl_ros::transformPointCloud(ref_frame, curTime, point_cloud_filtered, scene_cloud.header.frame_id, point_cloud_filtered, tfBuffer);

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
    seg.setDistanceThreshold(0.01);
    // Table should be perpendicular to the z axis (in the x-y plane)
    Eigen::Vector3f zAxis = Eigen::Vector3f(0,0,1);
    seg.setAxis(zAxis);
    seg.setEpsAngle(0.1);

    seg.setInputCloud(point_cloud_filtered.makeShared());
    seg.segment(*inliers, *coefficients);

    ROS_INFO("Segmented plane");

    PointCloud planeCloud;

    // Make new cloud with inliers only
    pcl::copyPointCloud(point_cloud_filtered, *inliers, planeCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr noPlane(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("noplane initial frame: %s", noPlane->header.frame_id.c_str());
    noPlane->header.frame_id = ref_frame;

    // New: Passthrough in z to remove anything below top of table

    float maxz = -999;
    for (auto point : planeCloud)
    {
        if(point.z > maxz)
        {
            maxz = point.z;
        }
    }
    float table_z = maxz;

    pcl::PassThrough<pcl::PointXYZ> passz;
    passz.setInputCloud(point_cloud_filtered.makeShared());
    passz.setFilterFieldName("z");
    passz.setFilterLimits(-999, maxz);
    passz.setFilterLimitsNegative (true);
    passz.filter(*noPlane);

    pcl::io::savePCDFile("noplane.pcd", *noPlane);
    pcl::toROSMsg(*noPlane, _noplane_cloud);
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
    float miny = 9999;

    for (auto point : noPlane->points)
        {
            avgX += point.x;
            avgY += point.y;
            avgZ += point.z;
            numpoints += 1;
            if(point.y < miny)
            {
                miny = point.y;
            }
        }
    // Hey5: we pick up objects from the right with hey5, so find rightmost point of object and standoff from there
    // average_point = pcl::PointXYZ(avgX/numpoints, miny, avgZ/numpoints);

    // Gripper: we pick at the center w/ gripper, so use all averages
    average_point = pcl::PointXYZ(avgX/numpoints, avgY/numpoints, avgZ/numpoints);

    // find offset between grasping fram and frame link moveit uses
    

    // TODO: find a way to decide orientation of pick point, either from shape, object identity, or both

    geometry_msgs::PoseStamped goal_pose;
    // Hey5: offsets in x and y because we move the wirst, not the hand: needed for alignment
    // goal_pose.pose.position.x = average_point.x - 0.1;
    // goal_pose.pose.position.y = average_point.y - 0.04;

    // Gripper: no offsets other than wrist to tool, we want the object to be in the middle
    goal_pose.pose.position.x = average_point.x + grip_to_wrist_tf.transform.translation.x;
    goal_pose.pose.position.y = average_point.y + grip_to_wrist_tf.transform.translation.y;
    goal_pose.pose.position.z = average_point.z + grip_to_wrist_tf.transform.translation.z + 0.08;

    // TODO: insert orientation here

    // Gripper
    tf2::Quaternion quat, transform_quat;
    tf2::fromMsg(grip_to_wrist_tf.transform.rotation, transform_quat);
    quat.setRPY(0, M_PI/8, 0);

    // quat.setRPY(0, 0, 0);
    quat = quat*transform_quat;
    quat.normalize();

    goal_pose.pose.orientation = tf2::toMsg(quat);

    goal_pose.header.frame_id = ref_frame;
    goal_pose.header.stamp = ros::Time::now();
    _pick_pose = goal_pose;
    ROS_INFO("Pick pose found");
    std::cout << "pose: " << std::endl << _pick_pose << std::endl;
    pick_point_pub.publish(_pick_pose);
    got_pick_pose = true;

    ROS_INFO("Publishing object pick height from table");
    std_msgs::Float32 object_pick_height;
    object_pick_height.data = average_point.z - table_z;
    object_height_pub.publish(object_pick_height);

    // resetSearch();
}

void CloudObjectFinder::resetSearch()
{
    ROS_INFO("CloudObjectFinder: Resetting, will need to be told what to look for again");
    desired_object_type = "";
    _detection_sub = _nh.subscribe("bounding_boxes", 5, &CloudObjectFinder::detectionCallback, this);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"homodeus_proc_point_cloud_perception_node");

    ros::NodeHandle nh;

    double frequency = 5;

    ROS_INFO("Creating cloud object finder");

    CloudObjectFinder finder(nh);
    ROS_INFO("Spinning to serve callbacks ...");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
     

    return 0;
}
