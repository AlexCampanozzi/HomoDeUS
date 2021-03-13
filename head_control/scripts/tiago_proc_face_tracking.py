#! /usr/bin/env python
import time

import rospy
import roslib
import actionlib
from geometry_msgs.msg import PoseStamped
import geometry_msgs
import math

from face_detection.msg import FacePosition
from face_detection.msg import FacePositions

class FaceTracking:
    def __init__(self):
        rospy.loginfo("Face tracking constructing")

        rospy.Subscriber('/pal_face/faces', FacePositions, self._head_callback, queue_size=5)
        self.pub = rospy.Publisher('tiago_head_controller', geometry_msgs.msg.PoseStamped, queue_size=5)


        # Collecting image settings
        self.img_width = 320 #rospy.get_param('processing_img_width')
        self.img_height = 240 #rospy.get_param('processing_img_height')

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.threshold = 1

        # Timestamp updated each time a face is detected
        self.timestamp = time.time()

    def _head_callback(self, detections):
        # TODO: Maybe this should go after the activation check?
        self.timestamp = time.time()
        rospy.loginfo("recieved face")
        main_face_x = 0
        main_face_y = 0
        main_face_dist = 1000000

        # Find the closest face to the image center (main face)
        for face in detections.faces:
            face_x, face_y = self._get_face_center_position(face)
            face_dist = self._distance_from_img_center(face_x, face_y)

            if (face_dist < main_face_dist):
                main_face_x = face_x
                main_face_y = face_y
                main_face_dist = face_dist

        # If the main face is inside the limit, don't move the head
        if main_face_dist < self.threshold:
            return

        poseStamped = geometry_msgs.msg.PoseStamped()

        poseStamped.pose.position.x = main_face_x
        poseStamped.pose.position.x = main_face_y

        rospy.loginfo("sent pose")
        self.pub.publish(poseStamped)

    def _distance_from_img_center(self, x, y):
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y)**2)

    def _get_face_center_position(self, face):
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)

        return x, y

if __name__ == "__main__":

    try:
        rospy.init_node('faceTracking', anonymous=False)
        faceTracking = FaceTracking()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass