#include "ros/ros.h"
#include "std_msgs/String.h"
#include <homodeus_arm_interface/ArmInterface.h>
#include <custom_msgs.msg/ArmData>

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
    //        rospy.Subscriber('/Proc_Arm_Mover', ArmMoves, self._arm_mover_callback, queue_size=5)
    ros::Subscriber _date_subscriber = _node.subscribe("arm_controler", 10, _arm_controler_callback);

    ros::Publisher _res_publisher = _node.advertise<Bool>("res_arm_controler",10);

    ros::spin(); //spin only once if using a while spinOnce()

    return 0;
}
