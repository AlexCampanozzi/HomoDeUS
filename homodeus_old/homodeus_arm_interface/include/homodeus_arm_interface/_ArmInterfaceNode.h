#ifndef ARMINTERFACENODE_H
#define ARMINTERFACENODE_H

#include <homodeus_arm_interface/ArmInterface.h>

#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


class ArmInterfaceNode: ArmInterface
{
    private:
        bool got_pick_pose = false;

        ros::NodeHandle nh;

        ros::Subscriber pick_pose_sub;

        geometry_msgs::PoseStamped pick_point;

        // Gripper client
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gac;
        control_msgs::FollowJointTrajectoryGoal close_gripper_goal;
        control_msgs::FollowJointTrajectoryGoal open_gripper_goal;

        void pickPoseCB(const geometry_msgs::PoseStampedConstPtr posestamped);
        trajectory_msgs::JointTrajectory openedGripper();
        trajectory_msgs::JointTrajectory closedGripper();
        // UNUSED ATM
        trajectory_msgs::JointTrajectory openedFingers();
        trajectory_msgs::JointTrajectory closedFingers();

    public:
        ArmInterfaceNode(ros::NodeHandle n);

        bool gotoGraspPrep();
        bool gotoRetreat();

        void closeHand();
};

#endif