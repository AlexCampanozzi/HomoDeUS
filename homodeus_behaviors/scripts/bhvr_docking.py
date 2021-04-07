#!/usr/bin/env python2

import rospy
import rospkg
import actionlib
from dock_charge_sm_msgs.msg import GoAndDockAction, GoAndDockGoal
from laser_servoing_msgs.msg import UndockAction, UndockGoal

from std_msgs.msg import Bool
from base_navigation.scripts.navigator import Navigator

class SimpleDock():
    def __init__ ( self ) :
        self.dock_checker_sub = rospy.Subscriber("/power/is_docked", Bool, self.is_docked_cb)
        self.is_docked = False

    def go_and_dock_client(self):
        goal = GoAndDockGoal()
        goal.use_current_pose = True
        self.dock_client = actionlib.SimpleActionClient("go_and_dock", GoAndDockAction)
        self.dock_client.wait_for_server()
        self.dock_client.send_goal(goal)
        rospy.loginfo("goal sent to go and dock server")

    def undocking_client(self):
        self.undock_client = actionlib.SimpleActionClient("undocker_server", UndockAction)
        self.dock_client.wait_for_server()
        self.dock_client.send_goal(goal)

        
    def is_docked_cb(self, is_docked):
        if (is_docked.data):
            self.dock_checker_sub.unregister()
            rospy.loginfo("the robot is docked!")
            quit()



if __name__ == '__main__':
    try:
        rospy.init_node('bhvr_docking')
        
        # Going to the docking station
        nav = Navigator()
        nav.registerLandmark("dock") # Will be done at the start of the scenario
        nav.goto(0,0,0) # Moving the robot for testing purposes

        nav.goToLandmark("dock")

        # Docking procedure
        sd = SimpleDock()
        sd.go_and_dock_client()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        nav.cancelAllGoto
