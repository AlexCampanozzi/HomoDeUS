#include <homodeus_arm_interface/ArmInterface.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <play_motion_msgs/PlayMotionGoal.h>

bool rdy2close = false;

void poseCB(const geometry_msgs::PoseStampedConstPtr posestamped)
{
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
    success = arm.moveToCartesian(x-0.3, y, z, roll, pitch, yaw);
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
    ros::Publisher finger_pub = n.advertise<trajectory_msgs::JointTrajectory>("/hand_controller/command", 5);
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
    

    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> ac("/play_motion", true);
    ac.waitForServer();
    
    ROS_INFO("arm_interface_node is now running!");

    //Prepare grasp
    play_motion_msgs::PlayMotionGoal goal;
    goal.motion_name = "prepare_grasp";
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(45.0));
    ROS_INFO("Ready to take object");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if(rdy2close)
        {
            finger_pub.publish(close_fingers);
            ROS_INFO("Closing fingers");
            rdy2close = false;
        }
    }

    // ArmInterface arm;
    // ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    // auto success = arm.moveToCartesian(0.4, -0.3, 0.26, -0.011, 1.57, 0.037);

    ros::waitForShutdown();
    return 0;
}

