import rospy

# sys.path.append(".")
from navigator import Navigator
from geometry_msgs.msg import PoseStamped

class HBBA_nav_listener:
    def __init__(self):
        self.navigator = Navigator()
        rospy.init_node('HBBA_nav_listener', anonymous=True)

    def callback(self, data):
        self.navigator.goto(data.pose.position.x, data.pose.position.y, data.pose.orientation.w)

    def listen(self):
        rospy.Subscriber('hbba_navgoal', PoseStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener = HBBA_nav_listener()
    listener.listen()
