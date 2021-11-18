#! /usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
from head_control.scripts.headController import headController
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import HomoDeUS_common_py.HomoDeUS_common_py as common  

BASE_MVMT_LIMIT = ""
MAX_REACHEABLE_RIGHT = 1.24
MAX_REACHEABLE_LEFT = -1.24
MAX_REACHEABLE_HEIGHT = 0.79
MAX_REACHEABLE_DOWN = -0.98


class ObjectTracking:
    def __init__(self, mode):
        rospy.Subscriber('bounding_boxes',BoundingBoxes, self._head_cb,queue_size= 5)
        #rospy.Subscriber('/proc_output_face_positions', FacePositions, self._head_callback, queue_size=5)
        rospy.Subscriber('desired_object', String, self._desired_obj_cb, queue_size=5)

        if mode == "remote":
            camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        else:
            camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
    
        self.img_center_x = camera_info.height // 2
        self.img_center_y = camera_info.width // 2

        #self.perception_pub = rospy.Publisher('object_bounding_box')
        #self.pub = rospy.Publisher('head_command', JointTrajectory, queue_size=2)
        self.head_controller = headController('head_command')
        self.pubObserver = rospy.Publisher('obs_tracking_object', Bool, queue_size=5)

        self.distance_threshold = 5
        self.desired_object = None
        self.pid_x = common.PID(self.img_center_x, K_P=-0.0010, K_I=-0.0001, K_D=-0.00005)
        self.pid_y = common.PID(self.img_center_y, K_P=-0.0010, K_I=-0.0001, K_D=-0.00005)

    def _desired_obj_cb(self, d_object):
        rospy.loginfo("desired object receive: " + str(d_object.data))
        self.desired_object = d_object.data

    def _head_cb(self, boxes):
        boxes = boxes.bounding_boxes
        # Find the closest face to the image center (main face)
        head_command = None
        if self.desired_object is not None:
            desired_obj_position = self._detect_desired_object(boxes)
            rospy.logerr(str(desired_obj_position))
            if desired_obj_position:
                head_command = self._center_desired_object(desired_obj_position)
            if head_command is not None:
                #Repeat the command since detection slow
                #TODO: adapt it according to the input rate
                for index in range(4):
                    self.head_controller.GotoPosition(head_command[0],head_command[1], 1.)
                    rospy.sleep(0.1)

    def _detect_desired_object(self,boxes):
        for box in boxes:
            if box.Class == self.desired_object:
                object_x_center = box.xmin + abs((box.xmax-box.xmin) // 2)
                object_y_center = box.ymin + abs((box.ymax-box.ymin) // 2)
                return object_x_center, object_y_center
    
    def _center_desired_object(self, obj_position):
        if not self._already_centered(obj_position):
            x_command = self.pid_x.get_next_command(obj_position[0])
            y_command = self.pid_y.get_next_command(obj_position[1])
            return [x_command, y_command]
        
    def _already_centered(self,obj_position):
        if self.distance_threshold > self._distance_from_img_center(obj_position):
            self.pubObserver.publish(True)
            return True
        else:
            return False

    def _distance_from_img_center(self, obj_position):
        answer = math.sqrt((self.img_center_x - obj_position[0])**2 + (self.img_center_y - obj_position[1])**2)
        return answer

if __name__ == "__main__":

    try:
        rospy.init_node('ObjectTracking', anonymous=False)
        mode = rospy.get_param('camera_mode', 'simul')
        ObjectTracking = ObjectTracking(mode)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
