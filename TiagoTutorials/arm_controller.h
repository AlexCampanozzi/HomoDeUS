// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

class ArmController
{
	private:
		string topic;

	public:
		ArmController();
		~ArmController();		
		int moveJoints(float torso, float J1, float J2, float J3, float J4, float J5, float J6, float J7);
		void moveCartesian(float x, float y, float z, float u, float v, float w);
}


