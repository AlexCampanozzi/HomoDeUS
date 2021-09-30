#include "ros/ros.h"
#include "std_msgs/String.h"
#include <homodeus_arm_interface/ArmInterface.h>


void _arm_controler_callback(int data){ //replace data by the type of the msg
        //data should be the position and orientation of the end effector and the pick or drop action
        //move_hand to position(data)
        //pick() or drop()
        //position of transport()
  }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle _node;

    ros::Subscriber _subscriber = _node.subscribe("arm_controler", 10, _arm_controler_callback);

    ros::spin();

    return 0;
}