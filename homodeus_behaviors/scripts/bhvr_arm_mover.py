#!/usr/bin/env python2
import rospy








class bhvr_arm_mover():
    def __init__(self):
        rospy.loginfo("initiating pick_up")
        rospy.Subscriber('/Proc_Arm_Mover', ArmMoves, self._arm_mover_callback, queue_size=5)


    def _arm_mover_callback(self, data):
    




    def wave():








    




