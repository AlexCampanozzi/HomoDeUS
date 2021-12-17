#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
import time
from std_msgs.msg import UInt16, Empty
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from custom_msgs.msg import TestDesire

class testManager:
    """
    This class is only useful for testing the reaction following a particular desire
    """
    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        self.sub            = rospy.Subscriber('select_desire',TestDesire, self.motv_cb, queue_size=5)
        self.sub            = rospy.Subscriber('remove_desire',UInt16, self.remove_cb, queue_size=5)
        
        # all desires id
        self.desire_track_object = 'test_objectTracking'
        self.desire_look_around = 'test_lookAround'
        self.desire_talking = 'test_talking'
        self.desire_keyword = 'test_keyword'
        self.desire_dialog = 'test_dialog'
        self.desire_pick = 'test_pick'
        self.desire_place = 'test_place'
        self.desire_approach_client = 'approach_client'
        

        self.current_desire = []
        rospy.wait_for_service("add_desires")

    def motv_cb(self, desire):
        rospy.logwarn('adding new desire ' + str(desire.desire_id))
        if desire.desire_id == 1:
            self.add(self.desire_track_object, "Track_object", "{object: '"+ desire.param + "'}")
        if desire.desire_id == 2:
            self.add(self.desire_look_around, "Look")
        if desire.desire_id == 3:
            self.add(self.desire_talking, "Talking", "{TtsText: '"+ desire.param + "'}")
        if desire.desire_id == 4:
            self.add(self.desire_keyword, "Keyword_detection", "{keyword: 'roboto'}")
        if desire.desire_id == 5:
            self.add(self.desire_dialog, "Dialoguing", "{context: '"+ desire.param + "'}")
        if desire.desire_id == 6:
            self.add(self.desire_pick, "Pick", "{object: '"+ desire.param + "'}")
        if desire.desire_id == 7:
            self.add(self.desire_place, "Place", "{object: '"+ desire.param + "'}")
        if desire.desire_id == 8:
            self.add(self.desire_approach_client, "approach_client")
        
    def remove_cb(self,desireNumber):

        if desireNumber.data==0:
            #remove all
            if self.current_desire:
                self.rem_desires.call(self.current_desire)
                del self.current_desire[:]
        else:
            self.remove(desireNumber.data)

    def remove(self,desireNumber):
        if desireNumber == 1:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_track_object])
                self.current_desire.remove(self.desire_track_object)
            else:
                rospy.logwarn("ObjectTracking desire wasn't even add")

        if desireNumber == 2:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_look_around])
                self.current_desire.remove(self.desire_track_object)
            else:
                rospy.logwarn("LookingAround desire wasn't even add")

        if desireNumber == 3:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_talking])
                self.current_desire.remove(self.desire_track_object)
            else:
                rospy.logwarn("Talking desire wasn't even add")
                self.rem_desires.call([self.desire_talking])
    
        if desireNumber == 4:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_keyword])
                self.current_desire.remove(self.desire_keyword)
            else:
                rospy.logwarn("KeywordDetection desire wasn't even add")
            
        if desireNumber == 5:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_dialog])
                self.current_desire.remove(self.desire_dialog)
            else:
                rospy.logwarn("Dialog desire wasn't even add")
            
        if desireNumber == 6:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_pick])
                self.current_desire.remove(self.desire_pick)
            else:
                rospy.logwarn("Pick desire wasn't even add")
            
        if desireNumber == 7:
            if self.desire_track_object in self.current_desire:
                self.rem_desires.call([self.desire_place])
                self.current_desire.remove(self.desire_place)
            else:
                rospy.logwarn("Place desire wasn't even add")
            
        if desireNumber == 8:
            if self.desire_approach_client in self.current_desire:
                self.rem_desires.call([self.desire_approach_client])
                self.current_desire.remove(self.desire_approach_client)
            else:
                rospy.logwarn("ApproachClient desire wasn't even add")

    def add(self,des_id, des_type, params=""):
        des = Desire()
        des.id          = des_id
        des.type        = des_type
        des.utility     = 6.0
        des.intensity   = 3.0
        if params:
            des.params      = params

        rospy.logwarn(des)
        self.add_desires.call([des])
        self.current_desire.append(des_id)


    def removeOnEvent(self, event):
        if event.desire_type == "Listening" and event.type == Event.ACC_ON:
            rospy.loginfo("REMOVING DESIRE LISTENING")
            self.rem_desires.call([event.desire])
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_test_all")

        node = testManager()
        # Note to self: see about using IW ruleset instead to remove desires
        #node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
