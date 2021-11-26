#! /usr/bin/env python


import rospy
import rosservice
import threading
from std_msgs.msg import Empty
from pal_startup_msgs.srv import StartupStop
from geometry_msgs.msg import PoseStamped, Pose
from base_navigation.scripts.navigator import Navigator
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import HomoDeUS_common_py.HomoDeUS_common_py as common


HEAD_STATE = '/head_controller/state'
HEAD_RESET = 'head_home_reset'
HEAD_CTRL_DEFAULT = '/head_controller/command'
HEAD_DISTANCED_CONTROL = 'tiago_head_controller'

TEST_TOPIC = 'test_head_controller'



class headController:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self,rate=0.1,initial_x= 0.0,initial_y=0.0,initial_duration=1.0, head_control_topic=HEAD_CTRL_DEFAULT, is_node = False):
        self.init_class_attributes(rate,initial_x,initial_y,initial_duration,head_control_topic)
        # Disabling the pal_head_manager to prevent unwanted head motion while moving the head
        self.startup_verification()
        if not is_node:
            self.sending_thread.start()

    def test_cb(self, data):
        # insert what you want to test here
        self.center_base()

    def home_cb(self, data):
        self.goto_position(repeat=False, x=0.0, y=0.0, duration=1.0)

    def distance_cb(self,data):
        self.goto_position(repeat=False, x=data.pose.position.x, y=data.pose.position.y, duration=1.0)
            
    def goto_position(self, repeat, x, y, duration=1.):
        """
        This method publishes a command to move the robot head in absolute
        x (float): The x position in the absolute frame that the robot must reach
        y (float): The y position in the absolute frame that the robot must reach
        """
        if repeat:
            self.set_new_pose(x,y,duration)
            self.start_repeated_sending()
        else:
            self.stop_repeated_sending()
            self.set_new_pose(x,y,duration)
            self.pub_commands.publish(self.command)

    def sendthread(self):
        while not rospy.is_shutdown():
            if self.repeat_sending:
                if self.change_command_lock.acquire():
                    self.pub_commands.publish(self.command)
                    self.change_command_lock.release()
            rospy.sleep(self.commands_rate)

    def set_rate(self, rate):
        self.rate = rate
    
    def stop_repeated_sending(self):
        self.repeat_sending = False
    
    def start_repeated_sending(self):
        self.repeat_sending = True

    def get_head_pose(self):
        pose = common.get_relative_pose('base_footprint','head_1_link')
        return common.convert_pose_quaternion_to_euler(pose)

    def get_head_angles(self):
        head_actual_position = rospy.wait_for_message(HEAD_STATE,JointTrajectoryControllerState)
        actual_x_position = head_actual_position.actual.positions[0]
        actual_y_position = head_actual_position.actual.positions[1]
        return [actual_x_position, actual_y_position]

    def set_new_pose(self,x, y, duration=1.):
        self.change_command_lock.acquire()
        self.points.positions = [x,y]
        self.points.time_from_start = rospy.Duration(duration)
        self.command.points[0] = self.points
        self.change_command_lock.release()

    def is_base_align(self):
        if self.nav is None:
            self.nav=Navigator()
        min_range = 0.20 # 10 degrees in rad
        pose_head = self.get_head_pose()
        return (abs(pose_head.orientation[2]) < min_range)

    def center_base(self):
        if self.nav is None:
            self.nav=Navigator()
        pose_head = self.get_head_pose()
        pose_base = common.convert_pose_quaternion_to_euler(self.nav.getCurPose())
        orientation = pose_base.orientation[2] + pose_head.orientation[2]
        head_actual_position = self.get_head_angles()
        self.goto_position(True,0.0, head_actual_position[1], duration=2.0)
        return self.nav.goto(pose_base.position.x, pose_base.position.y, orientation, blocking=True)

    def init_node_comm(self):
        rospy.Subscriber(HEAD_DISTANCED_CONTROL, PoseStamped, self.distance_cb)
        rospy.Subscriber(HEAD_RESET, Empty, self.home_cb)
        rospy.Subscriber(TEST_TOPIC, Empty, self.test_cb)  

    def init_class_attributes(self, rate, initial_x, initial_y, initial_duration, head_control_topic):
        self.commands_rate = rate
        self.sending_thread = threading.Thread(target=self.sendthread)
        self.change_command_lock = threading.Lock()
        self.nav = None


        # Publisher
        self.pub_commands = rospy.Publisher(head_control_topic, JointTrajectory, queue_size=5)

        # Flags
        self.repeat_sending = False
                
        # initial positions
        self.points = JointTrajectoryPoint()
        self.points.time_from_start = rospy.Duration(initial_duration)
        self.points.positions = [initial_x, initial_y]

        # init command
        self.command = JointTrajectory()
        self.command.joint_names = ["head_1_joint", "head_2_joint"]
        self.command.points.append(self.points)


    
    def startup_verification(self):
        service_list = rosservice.get_service_list()
        if '/pal_startup_control/stop' in service_list:
            try:
                rospy.wait_for_service('/pal_startup_control/stop', 2)
            except rospy.ROSException or rospy.ServiceException as e:
                rospy.logerr('Could not reach pal_startup_control/stop : %s', e.message)
            pal_stop = rospy.ServiceProxy('/pal_startup_control/stop', StartupStop)
            try:
                rospy.loginfo("disabling pal_head_manager.")
                pal_stop("head_manager")
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr('Could not stop head_manager: %s', e.message)

if __name__ == '__main__':

    rospy.init_node('headAction', anonymous=False)
    client = headController(is_node=True)
    client.init_node_comm()
    rospy.logwarn("=================== head_controller action ready =====================")
    rospy.spin()
