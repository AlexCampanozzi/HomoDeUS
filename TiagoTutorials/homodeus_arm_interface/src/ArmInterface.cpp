#include <homodeus_arm_interface/ArmInterface.h>

ArmInterface::ArmInterface() : _moveGroup("arm_torso")
{
    _planningTime = 5.0;
    _plannerId = "SBLkConfigDefault";

    _moveGroup.setPlannerId(_plannerId);

    _jointsNames = _moveGroup.getJoints();

    _moveGroup.setMaxVelocityScalingFactor(1.0);
}

bool ArmInterface::planTrajectory()
{
    ROS_INFO("ArmInterface::planTrajectory: Not implemented yet!");
    return false;
}

bool ArmInterface::planTrajectoryJ(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    
    _moveGroup.setStartStateToCurrentState();
    _moveGroup.setPlanningTime(_planningTime);

    bool success = static_cast<bool>(_moveGroup.plan(plan));
    
    return success;
}

void ArmInterface::setPlanningTime(float val)
{
    _planningTime = val;
}

void ArmInterface::setPlannerId(std::string Id)
{
    _plannerId = Id;
    _moveGroup.setPlannerId(_plannerId);
}

bool ArmInterface::moveTo()
{
    ROS_INFO("ArmInterface::moveTo: Not implemented yet!");
    return false;
}

bool ArmInterface::moveToJ(double torso, double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    _target_position["torso_lift_joint"] = torso;
    _target_position["arm_1_joint"] = j1;
    _target_position["arm_2_joint"] = j2;
    _target_position["arm_3_joint"] = j3;
    _target_position["arm_4_joint"] = j4;
    _target_position["arm_5_joint"] = j5;
    _target_position["arm_6_joint"] = j6;
    _target_position["arm_7_joint"] = j7;

    for (unsigned int i = 0; i < _jointsNames.size(); i++)
    {
        if (_target_position.count(_jointsNames[i]) > 0) 
            _moveGroup.setJointValueTarget(_jointsNames[i], _target_position[_jointsNames[i]]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan joints_plan;
    bool success = planTrajectoryJ(joints_plan);

    if (!success)
    {
        ROS_INFO("ArmInterface::moveToJ(): No plan found!");
        return false;
    }

    _moveGroup.move();

    return true;
}

ArmInterface::~ArmInterface()
{
    return;
}
