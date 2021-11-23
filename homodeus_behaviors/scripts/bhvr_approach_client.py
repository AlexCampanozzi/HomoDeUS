#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from custom_msgs.msg import FacePositions, FacePosition
from sensor_msgs.msg import LaserScan
import HomoDeUS_common_py as common
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from image_geometry import StereoCameraModel

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String

from base_navigation.scripts.navigator import Navigator
from cv_bridge import CvBridge, CvBridgeError
import tf

import numpy as np


class ApproachClient():
    def __init__(self):
        self.bridge = CvBridge()
        self.cameraModel = StereoCameraModel()
        self.depth_image = None
        self.approach_dist = 1.6
        self.navigator = Navigator()

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/proc_output_face_positions', FacePositions, self._face_callback, queue_size=5)
        rospy.Subscriber('/xtion/depth_registered/image_raw', Image, self._camera_callback, queue_size=5)
        self.tolerance = 0.15


        self.vel_publisher = rospy.Publisher("/mobile_base/cmd_vel", Twist, queue_size=5)
        self.pub = rospy.Publisher('tiago_head_controller', PoseStamped, queue_size=5)

        ## movement test

        poseStamped = PoseStamped()

        x = 0
        y = 0
        poseStamped.pose.position.x = x
        poseStamped.pose.position.y = y
        self.pub.publish(poseStamped)

        self.navigator.gotoLandmark("kitchenEntrance")


    def _face_callback(self, detections):

        if self.depth_image is not None and self.navigator.goalFinished:

            face_depth_view = self.depth_image[detections.faces[0].y: detections.faces[0].y + detections.faces[0].height, 
                                            detections.faces[0].x: detections.faces[0].x + detections.faces[0].width]

            face_dist = np.nanmin(face_depth_view)

            rospy.loginfo(face_dist)

            if face_dist > self.approach_dist + self.tolerance*self.approach_dist and not np.isnan(face_dist):

                # considering that the face is centered in the optical frame
                face_point = np.array([0, 0, face_dist])
                dist_to_approach = face_dist - self.approach_dist
                approach_point = np.array([0, 0, dist_to_approach])

                point = PointStamped()
                point.point.x = approach_point[0]
                point.point.y = approach_point[1]
                point.point.z = approach_point[2]
                point.header.stamp = rospy.Time(0)
                point.header.frame_id = "/xtion_rgb_optical_frame"

                map_point = self.tf_listener.transformPoint("/map", point)

                self.navigator.goto(map_point.point.x, map_point.point.y, np.pi-np.arctan(map_point.point.x/map_point.point.y))
                rospy.loginfo("approaching detected client")


    def _camera_callback(self, image):

        self.depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")

        


if __name__ == "__main__":

    try:
        rospy.init_node('approachClient', anonymous=False)
        approachClient = ApproachClient()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
