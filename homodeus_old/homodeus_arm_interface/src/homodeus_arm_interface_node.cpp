#include <homodeus_arm_interface/ArmInterface.h>

// Code to test the arm interface
int main(int argc, char **argv)
{
    std::string controlType;

    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~");

    ArmInterface arm;

    ROS_INFO("arm_interface_node is now running!");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    n.getParam("control_type", controlType);

    bool success = false;
            
    if (controlType.compare("j") == 0)
    {
        ROS_INFO("arm_interface_node: will attempt to move the arm in joints space.");
        success = arm.moveToJ(0.0, 2.7, 0.2, -2.1, 2.0, 1.0, -0.8, 0.0);
    }
    
    else if (controlType.compare("c") == 0)
    {
        ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
        success = arm.moveTo(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);
    }

    else
        ROS_ERROR("arm_interface_node: wrong control type! arm_interface_node only takes j for joints space or c for cartesian space!");

    if (success)
        ROS_INFO("arm_interface_node: succeeded!");
            
    else
        ROS_INFO("arm_interface_node: failed!");

    ros::waitForShutdown();
    return 0;
}
