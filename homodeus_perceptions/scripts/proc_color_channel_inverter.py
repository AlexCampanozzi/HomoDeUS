#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('/bgr_image_raw', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/xtion/rgb/image_raw",Image,self.callback)

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
        self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()
