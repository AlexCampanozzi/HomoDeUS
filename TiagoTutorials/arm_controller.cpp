#include "arm_controller.h"

ArmController::ArmController()
{
	ros::init(argc, argv, "plan_arm_torso_fk");
	ros::init(argc, argv, "plan_arm_torso_ik");

	ros::NodeHandle nh;

	topic = " ";
}

ArmController::~ArmController()
{
}

ArmController::moveJoints(float torso, float J1, float J2, float J3, float J4, float J5, float J6, float J7)
{
	//ros::Time start = ros::Time::now();
	std::map<std::string, double> target_position;

	target_position["torso_lift_joint"] = torso;
	target_position["arm_1_joint"] = J1;
	target_position["arm_2_joint"] = J2;
	target_position["arm_3_joint"] = J3;
	target_position["arm_4_joint"] = J4;
	target_position["arm_5_joint"] = J5;
	target_position["arm_6_joint"] = J6;
	target_position["arm_7_joint"] = J7;

	//ros::AsyncSpinner spinner(1);
	//spinner.start();

	std::vector<std::string> torso_arm_joint_names;

	//select group of joints
	moveit::planning_interface::MoveGroup group_arm_torso("arm_torso");

	//choose your preferred planner
	group_arm_torso.setPlannerId("SBLkConfigDefault");

	torso_arm_joint_names = group_arm_torso.getJoints();

	group_arm_torso.setStartStateToCurrentState();
	group_arm_torso.setMaxVelocityScalingFactor(1.0);


	for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
	if ( target_position.count(torso_arm_joint_names[i]) > 0 )
	{
	  ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
	  group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
	}

	moveit::planning_interface::MoveGroup::Plan my_plan;
	group_arm_torso.setPlanningTime(5.0);
	bool success = group_arm_torso.plan(my_plan);

	if ( !success )
	throw std::runtime_error("No plan found");

	ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

	group_arm_torso.move;
	//spinner.stop();
	return EXIT_SUCCESS;
}


