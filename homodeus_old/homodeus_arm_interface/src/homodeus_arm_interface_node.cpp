#include <homodeus_arm_interface/ArmInterface.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

bool rdy2close = false;
geometry_msgs::PoseStamped pick_point;

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

trajectory_msgs::JointTrajectory closedGripper()
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

trajectory_msgs::JointTrajectory openedGripper()
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

void poseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
    pick_point = *posestamped;
    bool success = false;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(posestamped->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = posestamped->pose.position.x;
    auto y  = posestamped->pose.position.y;
    auto z  = posestamped->pose.position.z;
    
    ArmInterface arm;
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // auto success = arm.moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);
    success = arm.moveToCartesian(x-0.2, y, z, roll, pitch, yaw);
    ros::Duration(1).sleep();
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first waypoint");
        success = arm.moveToCartesian(x, y, z, roll, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: succeeded!");
        rdy2close = true;
    }
    else
        ROS_INFO("arm_interface_node: failed!");
}

// Code to use the arm interface
int main(int argc, char **argv)
{
    std::string controlType;

    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~"); 

    ros::Subscriber pick_pose_sub = n.subscribe("/pick_point", 5, poseCB);

    // hey5
    // auto close_effector = closedFingers();
    // auto controller_topic = "/hand_controller/command";

    // gripper
    auto close_effector = closedGripper();
    auto open_effector = openedGripper();

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> aac("/arm_controller/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gac("/gripper_controller/follow_joint_trajectory", true);
    aac.waitForServer();
    gac.waitForServer();

    control_msgs::FollowJointTrajectoryGoal close_gripper_goal;
    close_gripper_goal.trajectory = close_effector;

    control_msgs::FollowJointTrajectoryGoal open_gripper_goal;
    open_gripper_goal.trajectory = open_effector;

    ROS_INFO("arm_interface_node is now running!");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Go to approach pose right away when node is launched
    // TODO make that happen in callback instead to avoid breaking in hbba
    // ROS_INFO("Going to grasp preparation pose");
    ArmInterface arm;
    bool success;
    success = arm.moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = arm.moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    success = arm.moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));

    if (success)
    {
        ROS_INFO("Now at grasp preparation pose");
    }
    else
    {
        ROS_INFO("Failed to go to grasp preparation pose in time");
    }

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
        if(rdy2close)
        {
            gac.sendGoal(close_gripper_goal);
            ROS_INFO("Closing fingers");
            rdy2close = false;
            ros::Duration(1).sleep();
            ROS_INFO("Moving to standoff point");
            ArmInterface arm;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(pick_point.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            auto x  = pick_point.pose.position.x;
            auto y  = pick_point.pose.position.y;
            auto z  = pick_point.pose.position.z;
            auto success = arm.moveToCartesian(x-0.1, y, z+0.2, roll, pitch, yaw);
            if (success)
            {
                ROS_INFO("arm_interface_node: succeeded!");
            }
            else
                ROS_INFO("arm_interface_node: failed!");
        }
    }

    // ArmInterface arm;
    // ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // auto success = arm.moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);

    ros::waitForShutdown();
    return 0;
}