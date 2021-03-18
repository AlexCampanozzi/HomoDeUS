#!/usr/bin/env python

# Simple program that publishes geometry_msgs/Twist messages on the /mobile_base_controller/cmd_vel topic  to move the base 
# Executes a spiral that slows and reverses.

import rospy
from geometry_msgs.msg import Twist


def talker():
    vel_msg = Twist()
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('base_cmds', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    vel_msg.linear.x = 1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 1
    mode = int(input("Enter desired mode (0 for default spiral, 1 to enter vel values): "))

    if mode is 0:
        pass
    elif mode is 1:
        velx = float(raw_input("Enter desired linear velocity: "))
        angz = float(raw_input("Enter desired agular velocity: "))
        vel_msg.linear.x = velx
        vel_msg.angular.z = angz
    else:
        print("Error: invalid mode entered")
        return

    while not rospy.is_shutdown():
        if mode is 0:
            vel_msg.linear.x -= 0.005
            vel_msg.angular.z -= 0.005
            if vel_msg.linear.x < -1:
                vel_msg.linear.x = -1
                vel_msg.angular.z = -1
        rospy.loginfo(vel_msg)
        pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
