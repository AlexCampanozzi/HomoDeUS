#include "ros/ros.h"
#include "std_msgs/String.h"
#include <arm_interface/ArmInterface.h>
#include <custom_msgs.msg/ArmData>

void _arm_controler_callback(ArmData data){ 
    
    bool success = false;

    if (data.type.compare("joint") == 0)
    {
        rospy.loginfo("arm_interface_node: will attempt to move the arm in joints space.");
        success = arm.moveToJoint(data.a, data.b, data.c, data.d, data.e, data.f, data.g, data.h);
    }

    else if (data.type.compare("coords") == 0)
    {
        rospy.loginfo("arm_interface_node: will attempt to move the arm in cartesian space.");
        success = arm.moveToCartesian(data.a, data.b, data.c, data.d, data.e, data.f);
    }

    else
    {
        ROS_ERROR("arm_interface_node: wrong control type! arm_interface_node only takes joint for joints space or coords for cartesian space!");
    }
    
    if (success)
        rospy.loginfo("arm_interface_node: succeeded!");

    else
        rospy.loginfo("arm_interface_node: failed!");

    ros::waitForShutdown();
    return 0;
    
    
    
    //replace data by the type of the msg

        //data should be the position and orientation of the end effector and the pick or drop action
        //move_hand to position(data)
        //pick() or drop()
        //position of transport()
  }

int main(int argc, char **argv)
{
    std::string controlType;
    ros::init(argc, argv, "listener");
    ArmInterface arm;

    rospy.loginfo("arm_interface_node is now running!");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle _node;
    //        rospy.Subscriber('/Proc_Arm_Mover', ArmMoves, self._arm_mover_callback, queue_size=5)
    ros::Subscriber _date_subscriber = _node.subscribe("arm_controler", 10, _arm_controler_callback);

    ros::Publisher _res_publisher = _node.advertise<Bool>("res_arm_controler",10);

    return 0;
}
