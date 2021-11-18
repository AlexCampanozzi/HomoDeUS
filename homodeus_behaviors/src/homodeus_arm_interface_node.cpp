#include <homodeus_arm_interface/ArmInterfaceNode.h>

ArmInterfaceNode::ArmInterfaceNode(ros::NodeHandle n):
nh{n}, 
gac("/gripper_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Node init strated");
    pick_pose_sub = nh.subscribe("/pick_point", 5, &ArmInterfaceNode::poseCB, this);
    close_gripper_goal.trajectory = closedGripper();
    open_gripper_goal.trajectory = openedGripper();
    ROS_INFO("Waiting for gripper joint controller server...");
    gac.waitForServer();
    ROS_INFO("Found  gripper joint controller server");
    ROS_INFO("Node init done");
}

// UNUSED: Use these instead of the gripper ones for Hey5 hand
trajectory_msgs::JointTrajectory closedFingers()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(3);
    close_fingers.joint_names[0] = "hand_index_joint";
    close_fingers.joint_names[1] = "hand_mrl_joint";
    close_fingers.joint_names[2] = "hand_thumb_joint"; 
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(3);
    close_fingers.points[0].positions[0] = 0.80;
    close_fingers.points[0].positions[1] = 1.60;
    close_fingers.points[0].positions[2] = 0.85;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedFingers()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(3);
    close_fingers.joint_names[0] = "hand_index_joint";
    close_fingers.joint_names[1] = "hand_mrl_joint";
    close_fingers.joint_names[2] = "hand_thumb_joint"; 
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(3);
    close_fingers.points[0].positions[0] = 0;
    close_fingers.points[0].positions[1] = 0;
    close_fingers.points[0].positions[2] = 0;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::closedGripper()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(2);
    close_fingers.joint_names[0] = "gripper_left_finger_joint";
    close_fingers.joint_names[1] = "gripper_right_finger_joint";
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(2);
    close_fingers.points[0].positions[0] = 0.01;
    close_fingers.points[0].positions[1] = 0.01;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedGripper()
{
    trajectory_msgs::JointTrajectory open_fingers;
    open_fingers.joint_names.resize(2);
    open_fingers.joint_names[0] = "gripper_left_finger_joint";
    open_fingers.joint_names[1] = "gripper_right_finger_joint";
    open_fingers.points.resize(1);
    open_fingers.points[0].positions.resize(2);
    open_fingers.points[0].positions[0] = 0.04;
    open_fingers.points[0].positions[1] = 0.04;
    open_fingers.points[0].time_from_start = ros::Duration(0.5);
    return open_fingers;
}

void ArmInterfaceNode::poseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
    pick_point = *posestamped;
    got_pick_pose = true;
    bool success = false;

    ROS_INFO("Going to grasp preparation pose");
    success = gotoGraspPrep();
    if (success)
    {
        ROS_INFO("Now at grasp preparation pose");
    }
    else
    {
        ROS_INFO("Failed to go to grasp preparation pose in time, will still attempt rest of pick sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped->pose.position.x;
    auto y  = posestamped->pose.position.y;
    auto z  = posestamped->pose.position.z;
    
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // success = moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);
    success = moveToCartesian(x-0.2, y, z, roll, pitch, yaw);
    ros::Duration(1).sleep();
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first waypoint");
        success = moveToCartesian(x, y, z, roll, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: successfully moved to pick point, clsing gripper...");
        gac.sendGoalAndWait(close_gripper_goal, ros::Duration(2));
        ROS_INFO("Closed!");
    }
    else
        ROS_INFO("arm_interface_node: failed to go to pick point!");

    success = gotoRetreat();
    if (success)
    {
        ROS_INFO("arm_interface_node: successfully retreated from pick point.");
    }
    else
        ROS_INFO("arm_interface_node: failed to retreat from pick point!");

}

bool ArmInterfaceNode::gotoGraspPrep()
{
    bool success;
    success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));
    return success;
}

bool ArmInterfaceNode::gotoRetreat()
{
    ROS_INFO("Attempting retreat from pick_pose");
    if (got_pick_pose)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pick_point.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        auto x  = pick_point.pose.position.x;
        auto y  = pick_point.pose.position.y;
        auto z  = pick_point.pose.position.z;
        return moveToCartesian(x-0.1, y, z+0.2, roll, pitch, yaw);
    }
    else
    {
        ROS_INFO("Could not go to retreat point: no pick pose set");
        return false;
    }
}

// Code to use the arm interface
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~"); 

    ArmInterfaceNode arm_node(n);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}
