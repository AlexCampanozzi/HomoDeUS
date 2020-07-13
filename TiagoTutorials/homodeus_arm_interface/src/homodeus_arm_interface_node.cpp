#include <homodeus_arm_interface/ArmInterface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    ArmInterface arm;

    ROS_INFO("arm_interface_node is now running!");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("arm_interface_node will attempt to move the arm.");
            
    bool success = arm.moveToJ(0.0, 2.7, 0.2, -2.1, 2.0, 1.0, -0.8, 0.0);

    if (success)
        ROS_INFO("arm_interface_node succeeded!");
            
    else
        ROS_INFO("arm_interface_node failed!");

    ros::waitForShutdown();
    return 0;
}
