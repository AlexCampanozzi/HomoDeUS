#!/usr/bin/env python2
import rospy

from navigator import Navigator
from geometry_msgs.msg import PoseStamped
from base_navigation.msg import GoToResult
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class HBBA_nav_listener(Navigator):
    def __init__(self):
        Navigator.__init__(self)
        self.result_pub = rospy.Publisher("nav_result", GoToResult, queue_size=5)
        # navigator inits a node, therefore cant init one here
        # rospy.init_node('HBBA_nav_listener', anonymous=True)

    def callback(self, data):
        # Publish out the reult of the goto action, laong with current coordinates
        print data.pose.orientation
        result = GoToResult()
        result.result = self.goto(data.pose.position.x, data.pose.position.y, data.pose.orientation.w)
        curpose = self.getCurPose()
        result.x = curpose.position.x
        result.y = curpose.position.y
        result.t = euler_from_quaternion((curpose.orientation.x, curpose.orientation.y, curpose.orientation.z, curpose.orientation.w))[2]
        self.result_pub.publish(result)

    def listen(self):
        rospy.Subscriber('/hbba_nav_goal', PoseStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = HBBA_nav_listener()
        listener.listen()
    except rospy.ROSInterruptException:
        pass

