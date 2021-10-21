#include <homodeus_arm_interface/ArmInterface.h>


/* ArmInterface: Constructor

Description:    Initialize various parameters to communicate
                with the arm and the torso through MoveIt.

Inputs:         None

Outputs:        None
*/
ArmInterface::ArmInterface() : _moveGroup("arm_torso"), _ref_frame("base_footprint")
{
    // Using 5 seconds because it's a reasonable delay
    _planningTime = 5.0;
    _plannerId = "SBLkConfigDefault";

    _moveGroup.setPlannerId(_plannerId);

    _jointsNames = _moveGroup.getJoints();

    // Using a factor of 1.0 at first, we'll see if this value needs to be changed
    _moveGroup.setMaxVelocityScalingFactor(1.0);
}

/* ArmInterface: Alternative Constructor

Description:    Initialize various parameters to communicate
                with the arm and the torso through MoveIt,
                specifying which tf frame to use for cartesian.

Inputs:         ref_frame (type, std::string)
                    The tf frame in which points are to be taken for cartesian movement

Outputs:        None
*/
ArmInterface::ArmInterface(std::string ref_frame) : _moveGroup("arm_torso"), _ref_frame(ref_frame)
{
    // Using 5 seconds because it's a reasonable delay
    _planningTime = 5.0;
    _plannerId = "SBLkConfigDefault";

    _moveGroup.setPlannerId(_plannerId);

    _jointsNames = _moveGroup.getJoints();

    // Using a factor of 1.0 at first, we'll see if this value needs to be changed
    _moveGroup.setMaxVelocityScalingFactor(1.0);
}


/* ArmInterface: Trajectory Planner in Cartesian Space

Description:    This method uses MoveIt to plan a trajectory
                in cartesian space according to a target.

Inputs:         None

Outputs:        plan (type, moveit::planning_interface::MoveGroupInterface::Plan):
                    Reference to the plan generated by MoveIt.

                success (type, bool):
                    This method returns true if the planner was able to find a trajectory
                    and false otherwise.
*/
bool ArmInterface::planTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    _moveGroup.setStartStateToCurrentState();
    _moveGroup.setPlanningTime(_planningTime);

    bool success = static_cast<bool>(_moveGroup.plan(plan));


    return success;
}


/* ArmInterface: Trajectory Planner in Joints Space

Description:    This method uses MoveIt to plan a trajectory
                in joints space according to a target.

Inputs:         None

Outputs:        plan (type, moveit::planning_interface::MoveGroupInterface::Plan):
                    Reference to the plan generated by MoveIt.

                success (type, bool):
                    This method returns true if the planner was able to find a trajectory
                    and false otherwise.
*/
bool ArmInterface::planTrajectoryJ(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    _moveGroup.setStartStateToCurrentState();
    _moveGroup.setPlanningTime(_planningTime);

    bool success = static_cast<bool>(_moveGroup.plan(plan));

    return success;
}


/* ArmInterface: Set Planning Time

Description:    This method sets the time allowed to the planner to find
                a trajectory. The default value of the planning time is
                5.0 seconds.

Inputs:         value (type: float)
                    The new value for the planning time.

Outputs:        None
*/
void ArmInterface::setPlanningTime(float value)
{
    _planningTime = value;
}


/* ArmInterface: Set the Planner ID

Description:    This method sets the planner's name used by MoveIt to find
                a trajectory. The default value of the planner's ID is set
                to "SBLkConfigDefault".

Inputs:         Id (type, std::string):
                    The new planner's ID

Outputs:        None
*/
void ArmInterface::setPlannerId(std::string id)
{
    _plannerId = id;
    _moveGroup.setPlannerId(_plannerId);
}


/* ArmInterface: Move the Arm To a Desired Position in Cartesian Space

Description:    This method uses the planTrajectory() method to find a trajectory
                to a desired position in cartesian space and moves the arm if a
                plan is found.

Inputs:         x (type, double):
                    The X coordinate of the desired position.

                y (type, double):
                    The Y coordinate of the desired position.

                z (type, double):
                    The Z coordinate of the desired position.

                roll (type, double):
                    The roll of the desired position.

                pitch (type, double):
                    The pitch of the desired position.

                yaw (type, double):
                    The yaw of the desired position.

Outputs:        success (type, bool):
                    This method returns true if it was able to move the arm and
                    false otherwise.
*/
bool ArmInterface::moveToCartesian(double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::PoseStamped goalPose;
    goalPose.header.frame_id = _ref_frame;
    goalPose.pose.position.x = x;
    goalPose.pose.position.y = y;
    goalPose.pose.position.z = z;
    goalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    _moveGroup.setPoseReferenceFrame(_ref_frame);
    _moveGroup.setPoseTarget(goalPose);

    moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
    bool success = planTrajectory(cartesianPlan);

    if (!success)
    {
        ROS_INFO("ArmInterface::moveTo(): No plan found!");
        return false;
    }
    else
    {
        ROS_INFO("ArmInterface::moveTo(): Plan found!");
    }

    _moveGroup.move();

    return true;
}


/* ArmInterface: Move the Arm To a Desired Position in Joints Space

Description:    This method uses the planTrajectoryJ() method to find a trajectory
                to a desired position in joints space and moves the arm if a
                plan is found.

Inputs:         torso (type, double):
                    The desired position of the torso.

                j1 (type, double):
                    The desired position of the first joint of the arm.

                j2 (type, double):
                    The desired position of the second joint of the arm.

                j3 (type, double):
                    The desired position of the third joint of the arm.

                j4 (type, double):
                    The desired position of the fourth joint of the arm.

                j5 (type, double):
                    The desired position of the fifth joint of the arm.

                j6 (type, double):
                    The desired position of the sixth joint of the arm.

                j7 (type, double):
                    The desired position of the seventh joint of the arm.

Outputs:        success (type, bool):
                    This method returns true if it was able to move the arm and
                    false otherwise.
*/
bool ArmInterface::moveToJoint(double torso, double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    std::map<std::string, double> targetPosition;

    targetPosition["torso_lift_joint"] = torso;
    targetPosition["arm_1_joint"] = j1;
    targetPosition["arm_2_joint"] = j2;
    targetPosition["arm_3_joint"] = j3;
    targetPosition["arm_4_joint"] = j4;
    targetPosition["arm_5_joint"] = j5;
    targetPosition["arm_6_joint"] = j6;
    targetPosition["arm_7_joint"] = j7;

    for (unsigned int i = 0; i < _jointsNames.size(); i++)
    {
        if (targetPosition.count(_jointsNames[i]) > 0)
            _moveGroup.setJointValueTarget(_jointsNames[i], targetPosition[_jointsNames[i]]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan jointsPlan;
    bool success = planTrajectoryJ(jointsPlan);

    if (!success)
    {
        ROS_INFO("ArmInterface::moveToJ(): No plan found!");
        return false;
    }

    _moveGroup.move();

    return true;
}
