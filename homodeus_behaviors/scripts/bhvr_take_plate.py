#!/usr/bin/env python2
import rospy








class bhvr_arm_mover():
    def __init__(self):
        rospy.loginfo("initiating pick_up")
        rospy.Subscriber('/Proc_Arm_Mover', ArmMoves, self._arm_mover_callback, queue_size=5)


    def _arm_mover_callback(self, data):
        #data should be the position and orientation of the end effector and the pick or drop action
        #move_hand to position(data)
        #pick() or drop()
        #position of transport()


    def pick(): #close hand on object



    def drop(): #open hand to release object 



    def wave(): #series of position for the waving act








    




