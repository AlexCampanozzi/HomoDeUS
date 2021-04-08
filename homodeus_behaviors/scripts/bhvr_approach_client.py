#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from custom_msgs.msg import FacePositions, FacePosition
from sensor_msgs.msg import LaserScan
import HomoDeUS_common_py as common

class ClientApproach(self):
    def __init__(self):
        self.vel_publisher = rospy.Publisher("/mobile_base/cmd_vel", Twist, queue_size=5)
        self.target_box_size = 80000 # actual number TBD
        self.tolerance = 500 # actual number TBD
        # We are looking to be around 1.6-2m from the client

        self.dif_to_vel_factor = 0.0001 # actual number TBD

        # It might be a good idea to use Depth from rgbd the laser scan as supplementary information input sources

        # Copied from face tracking
        camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        #camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)

        self.img_height = camera_info.height
        self.img_width = camera_info.width

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.scan_listener = rospy.Subscriber("bhvr_input_scan", LaserScan, self.scanCB, queue_size=5)

        # To make getting scan params a one-shot
        self.got_scan_params = False

        self.too_close = False

    def approach(self):
        self.boxes_listener = rospy.Subscriber('bhvr_input_faces', FacePositions, self.facesCB)

    def stopApproach(self):
        self.boxes_listener.unregister()

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

        if common.equalWithinTolerance(face_size, self.target_box_size, self.tolerance):
            # Then we are within tolerance and don't need to do anything
            return
        else:
            # >0: too big -> too close | <0: too small -> too far
            size_diff = face_size - self.target_box_size

            # TODO: if client is far enough, use goto to get closer: ~3m ish threshold for vel command?

            command = Twist()
            # invert sign so we move in right direction 
            command.linear.x = -1*size_diff * self.dif_to_vel_factor
            # or use self.min_range as a distance straight up, maybe?
            
            # if too close to stuff back up so rotation or goto can get us around obstacle
            if command.linear.x > 0 and self.too_close:
                command.linear.x = -0.01

            self.vel_publisher.publish(command)

            # Coding this assuming head_tracking will rotate base to center Client: only sending linear commands

    def scanCB(self, scan):
        scan = LaserScan()
        if self.got_scan_params == False:
            self.start_angle = scan.angle_min
            self.stop_angle = scan.angle_max
            self.angle_increment = scan.angle_increment
            self.angle_range = self.stop_angle - self.start_angle
            self.got_scan_params = True
        else:
            # commented because min seems stuck at 0.05 for some reason. Probably ctaching a bit of itself
            # if scan.range_min < 0.5:
            #     # too close to stuff
            #     self.too_close = True
            # else:
            #     self.too_close = False

            # Take the ranges only from the 3rd in front to decide what to do
            half_third = 0.33 / 2
            points = len(scan.ranges)
            midway_point = round(points/2)
            self.max_range = 0
            self.min_range = 25 #The max range as start value since we look for lower 
            for i in range(midway_point - half_third, midway_point + half_third):
                self.max_range = scan.ranges[i] if scan.ranges[i] > self.max_range
                self.min_range = scan.ranges[i] if scan.ranges[i] < self.min_range

            # TODO: some algo to decide what is a person vs not
