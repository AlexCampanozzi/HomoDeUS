#include "arm_controller.h"

void main()
{

ArmController arm();
float torso = 0;
float J1 = 2.7;
float J2 = 0.2;
float J3 = -2.1;
float J4 = 2.0;
float J5 = 1.0;
float J6 = -0.8;
float J7 = 0;
arm.moveJoints(torso, J1, J2, J3, J4, J5, J6, J7)



ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

spinner.stop();
}
