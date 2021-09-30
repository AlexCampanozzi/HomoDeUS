#!/usr/bin/env python2
import rospy
from custom_msgs.msg import ArmData

class bhvr_take_plate():
    def __init__(self):
        rospy.loginfo("initiating take_plate")

        self.input_bhvr_goal = rospy.Subscriber("/bhvr_input_goal_take_plate",data_class=ArmData,callback=self._take_plate_callback,queue_size=10)
        self.input_bhvr_result = rospy.Subscriber("/res_arm_controler",data_class=Bool,callback=self._res_callback,queue_size=10)


        self.output_take_plate = rospy.Publisher("/arm_controler", data_class=ArmData, queue_size=10)
        self.output_bhvr_result = rospy.Publisher("/bhvr_output_res_take_plate", Bool, queue_size=10)


    def _take_plate_callback(int data): #replace data by the type of the msg
        #data should be the position and orientation of the end effector and the pick or drop action
        self.output_take_plate.publish(True) #replace true by real data
        #move_hand to position(data)
        #pick() or drop()
        #position of transport()

    def _res_callback(Bool res) : #transmit the result of the cpp node to the goals
        self.output_bhvr_result.publish(res)










    




