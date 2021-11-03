#include <homodeus_arm_interface/ArmInterface.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>

void poseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
    bool success = false;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped->pose.position.x;
    auto y  = posestamped->pose.position.y;
    auto z  = posestamped->pose.position.z;
    
    ArmInterface arm;
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // auto success = arm.moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);
    success = arm.moveToCartesian(x-0.4, y, z, roll, pitch, yaw);
    success = arm.moveToCartesian(x, y, z, roll, pitch, yaw);
    

    if (success)
        ROS_INFO("arm_interface_node: succeeded!");

    else
        ROS_INFO("arm_interface_node: failed!");
}

// Code to use the arm interface
int main(int argc, char **argv)
{
    std::string controlType;

    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~");

    ros::Subscriber pick_pose_sub = n.subscribe("/pick_point", 5, poseCB);

    ROS_INFO("arm_interface_node is now running!");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    // ArmInterface arm;
    // ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // auto success = arm.moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);

    ros::waitForShutdown();
    return 0;
}
