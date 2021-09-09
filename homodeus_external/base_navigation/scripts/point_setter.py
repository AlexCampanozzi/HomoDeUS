#!/usr/bin/env python

# =============================================================================
# Simple program that publishes geometry_msgs/PoseStamped messages on the
# /move_base_simple/goal topic to move the robot to a set point int the map
# =============================================================================

import rospy
from geometry_msgs.msg import PoseStamped


class Navigator:
    def __init__(self):
        self.pose_msg = PoseStamped()
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.init_node('base_cmds', anonymous=True)

    def goto(self, posX, posY, oriZ):
        self.pose_msg.header.seq = 0  # Dunno what this is
        self.pose_msg.header.stamp = rospy.get_rostime()  # Stamp current time
        self.pose_msg.header.frame_id = "map"  # always navigating in map frame
        self.pose_msg.pose.position.x = x
        self.pose_msg.pose.position.y = y
        self.pose_msg.pose.position.z = 0  # 2D: z is always 0
        self.pose_msg.pose.orientation.x = 0  # 2D: always rotated only about z
        self.pose_msg.pose.orientation.y = 0  # 2D: always rotated only about z
        self.pose_msg.pose.orientation.z = 1  # 2D: always rotated only about z
        self.pose_msg.pose.orientation.w = w
        self.pub.publish(pose_msg)

def setter():
    pose_msg = PoseStamped()
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('base_cmds', anonymous=True)
    x = float(input("Enter desired x position: "))
    y = float(input("Enter desired y position: "))
    w = float(input("Enter desired orientation: "))
    pose_msg.header.seq = 0  # Dunno what this is
    pose_msg.header.stamp = rospy.get_rostime()  # Stamp current time
    pose_msg.header.frame_id = "map"  # always navigating in map frame
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = 0  # 2D: z is always 0
    pose_msg.pose.orientation.x = 0  # 2D: always rotated only about z
    pose_msg.pose.orientation.y = 0  # 2D: always rotated only about z
    pose_msg.pose.orientation.z = 1  # 2D: always rotated only about z
    pose_msg.pose.orientation.w = w
    pub.publish(pose_msg)


if __name__ == '__main__':
    try:
        setter()
    except rospy.ROSInterruptException:
        pass
