#include <drop_spot_perception/proc_drop_spot_perception.h>

DropSpotFinder::DropSpotFinder(ros::NodeHandle& nh): _nh(nh)
{
    image_info_sub  = _nh.subscribe("/xtion/rgb/camera_info", 5, &DropSpotFinder::imageInfoCallback, this);
    object_height_sub = _nh.subscribe("/object_pick_height", 1, &DropSpotFinder::objectHeightCallback, this);
    _cloud_sub = _nh.subscribe("points", 5, &DropSpotFinder::cloudCallback, this);
    // trigger_sub = _nh.subscribe("/proc_drop_point_trigger", 1, &DropSpotFinder::triggerCallback, this);
    tfListenerPtr = new tf2_ros::TransformListener(tfBuffer);

    drop_point_pub = _nh.advertise<geometry_msgs::PoseStamped>("/drop_point", 5);
    plane_cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/drop_plane_cloud", 5);
}

// void DropSpotFinder::triggerCallback(const std_msgs::EmptyConstPtr& nothing)
// {
//     _cloud_sub = _nh.subscribe("/xtion/depth_registered/points", 5, &DropSpotFinder::cloudCallback, this);
// }

void DropSpotFinder::imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& info)
{
    camera_info = *info;
    camera_model.fromCameraInfo(camera_info);
    image_info_sub.shutdown();
    got_cam_info = true;
    ROS_INFO("Got camera info");
    std::cout << "Info: " << camera_info << std::endl;
}

void DropSpotFinder::objectHeightCallback(const std_msgs::Float32ConstPtr& height)
{
    ROS_INFO("Received object pick height");
    object_height = height->data;
}

void DropSpotFinder::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!got_cam_info)
    {
        // then we cant transform pixels to coords yet, retry later
        ROS_INFO("Did not have cam_info when attempting to process cloud, will try again on next CB");
        return;
    }
    _cloud_sub.shutdown();

    pcl::fromROSMsg(*msg, scene_cloud);
    // Put the worked cloud in the robot's frame so operation directions make sense to us
    auto curTime = ros::Time::now();
    auto curTransform = tfBuffer.lookupTransform(ref_frame, scene_cloud.header.frame_id, curTime, ros::Duration(10));
    pcl_ros::transformPointCloud(ref_frame, curTime, scene_cloud, scene_cloud.header.frame_id, scene_cloud, tfBuffer);

    ROS_INFO("Moved to base_link frame");

    // Keep only points above floor
    PointCloud scene_cloud_filtered;

    ROS_INFO("Filtering to keep only points above the floor");
    pcl::PassThrough<pcl::PointXYZ> passx;
    passx.setInputCloud(scene_cloud.makeShared());
    passx.setFilterFieldName("z");
    passx.setFilterLimits(0.2, 100);
    // passx.setFilterLimitsNegative (true);
    passx.filter(scene_cloud_filtered);
    scene_cloud = scene_cloud_filtered;

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

    seg.setInputCloud(scene_cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    ROS_INFO("Segmented plane");

    // Make new cloud with inliers only
    PointCloud planeCloud;
    pcl::copyPointCloud(scene_cloud, *inliers, planeCloud);

    pcl::toROSMsg( planeCloud, plane_cloud);
    plane_cloud.header.frame_id = ref_frame;
    plane_cloud_pub.publish(plane_cloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr noPlane(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("noplane initial frame: %s", noPlane->header.frame_id.c_str());
    noPlane->header.frame_id = ref_frame;


    // find extremities while we're going through cloud

    float maxz = -999;
    float miny = 9999;
    float maxy = -999;
    float minx = 9999;
    for (auto point : planeCloud)
    {
        if(point.z > maxz)
        {
            maxz = point.z;
        }
        if (point.y < miny){
            miny = point.y;
        }
        if (point.y > maxy){
            maxy = point.y;
        }
        if (point.x < minx){
            minx = point.x;
        }
    }
    extremities table_zone(miny, maxy);
    ROS_INFO("Table y-bounds: min: %f, mac: %f", miny, maxy);
    float table_edge_x = minx;
    float table_z = maxz;

    pcl::PassThrough<pcl::PointXYZ> passz;
    passz.setInputCloud(scene_cloud.makeShared());
    passz.setFilterFieldName("z");
    passz.setFilterLimits(-999, maxz);
    passz.setFilterLimitsNegative (true);
    passz.filter(*noPlane);


    // Segment objects on table to find widths whithin which we cannot drop

    // Extract clusters to separate each object
    // Creating the KdTree object for the search method of the extraction
    ROS_INFO("Separating objects on table into distinct clouds");
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(noPlane);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.022); // 25 mm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(noPlane);
    ROS_INFO("Extracting cluster indices");
    ec.extract(cluster_indices);

    // Go through set of indices to create new clouds
    std::vector<PointCloud::Ptr> objectClouds;
    int j = 0;
        
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*noPlane)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        objectClouds.push_back(cloud_cluster);

        j++;
    }

    // Find outer bounds of objects: we cannot drop within those
    std::vector<extremities> object_zones;
    float min = 9999;
    float max = -999;
    for (auto cloud : objectClouds){
        min = 9999;
        max = -999;
        for (auto point: cloud->points){
            if (point.y < min){
                min = point.y;
            }
            if (point.y > max){
                max = point.y;
            }
        }
        object_zones.push_back(extremities(min, max));
        ROS_INFO("Added object with y-bounds: min: %f, mac: %f", min, max);
    }

    // Generate a point in y ot drop the object
    bool drop_y_found = false;
    float initial_search_y = 0;
    float search_y = 0;
    float search_step = 0.01;
    float search_direction = 1;
    int search_count = 0;
    bool search_is_whithin_object = false;
    while (!drop_y_found)
    {
        search_y = initial_search_y + search_direction*search_count*search_step;
        ROS_INFO("Current search_y: %f", search_y);
        for (auto zone : object_zones)
        {
            // Search with a 5 cm buffer
            if (zone.isWithinWtihTol(search_y, 0.05))
            {
                ROS_INFO("Search_y is within bounds of zone with min=%f and max=%f with tol=%f", zone.min, zone.max, 0.05);
                search_is_whithin_object = true;
                break;
            }
        }
        if (search_is_whithin_object){
            search_count += 1;
            ROS_INFO("Search count: %d", search_count);
            if(!table_zone.isWithin(initial_search_y + search_direction*search_count*search_step))
            {
                if (search_direction < 0)
                {
                    ROS_ERROR("Failed to find a free spot on table to put object");
                    return;
                }
                else
                {
                    ROS_INFO("Changing search direction");
                    search_direction *= -1;
                    search_count = 0;
                }
            }
            // reset for next attempt
            search_is_whithin_object = false;
        }
        else
        {
            ROS_INFO("Found drop_y");
            drop_y_found = true;
        }
    }
    auto drop_y = search_y;

    auto grip_to_wrist_tf =  tfBuffer.lookupTransform("gripper_grasping_frame", "arm_tool_link", ros::Time(0));

    geometry_msgs::PoseStamped goal_pose;

    // Gripper: no offsets other than wrist to tool, we want the object to be in the middle
    // edge - tool offset + depth to go to
    goal_pose.pose.position.x = table_edge_x + grip_to_wrist_tf.transform.translation.x + 0.10;
    goal_pose.pose.position.y = drop_y + grip_to_wrist_tf.transform.translation.y;
    goal_pose.pose.position.z = table_z + object_height + grip_to_wrist_tf.transform.translation.z + 0.08;

    // TODO: insert orientation from pick here

    // Gripper orientation has to match pick form proc_point_cloud_perception.cpp
    tf2::Quaternion quat, transform_quat;
    tf2::fromMsg(grip_to_wrist_tf.transform.rotation, transform_quat);
    quat.setRPY(0, M_PI/8, 0);

    // quat.setRPY(0, 0, 0);
    quat = quat*transform_quat;
    quat.normalize();

    goal_pose.pose.orientation = tf2::toMsg(quat);

    goal_pose.header.frame_id = ref_frame;
    goal_pose.header.stamp = ros::Time::now();
    _drop_pose = goal_pose;
    ROS_INFO("Drop pose found");
    std::cout << "pose: " << std::endl << _drop_pose << std::endl;
    drop_point_pub.publish(_drop_pose);
    got_drop_pose = true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"homodeus_proc_point_cloud_perception_node");

    ros::NodeHandle nh;

    double frequency = 5;

    ROS_INFO("Creating cloud object finder");

    DropSpotFinder finder(nh);
    ROS_INFO("Spinning to serve callbacks ...");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
     

    return 0;
}
