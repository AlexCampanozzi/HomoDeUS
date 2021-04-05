#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from custom_msgs.msg import FacePositions, FacePosition

class ClientApproach(self):
    def __init__(self):
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.target_box_size = 80000 # actual number TBD
        self.tolerance = 500 # actual number TBD

        # Copied from face tracking
        camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        #camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)

        self.img_height = camera_info.height
        self.img_width = camera_info.width

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

    def approach(self):
        rospy.Subscriber('bhvr_input_face_boxes', FacePositions, self.facesCB)

    # Following 2 methods are ripped directly from face_tracking. Move to common maybe?
    def _distance_from_img_center(self, x, y):
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y)**2)

    def _get_face_center_position(self, face):
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)
        return x, y

    def facesCB(self, detections):
        for face in detections.faces:
            face_x, face_y = self._get_face_center_position(face)
            face_dist_from_center = self._distance_from_img_center(face_x, face_y)

            if (face_dist_from_center < main_face_dist_from_center):
                main_face_x = face_x
                main_face_y = face_y
                main_face_dist_from_center = face_dist_from_center
                main_face_width = face.width
                main_face_height = face.height

        face_size = main_face_width x main_face_width
        # Will need fuzzy equal here to see if we move at all
