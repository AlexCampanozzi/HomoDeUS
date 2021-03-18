import rospy
import actionlib
import traceback
import pal_interaction_msgs.msg
from std_msgs.msg import String, Bool
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Talking_module:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self, language = "en_GB", text="Welcome my friends"):

        self.language = language
        self.talking_text = text
        # Goal input
        self.input_bhvr_goal = rospy.Subscriber("/bhvr_input_goal_talking",data_class=pal_interaction_msgs.msg.TtsText,callback=self.action_Cb,queue_size=10)
        
        # Output
        self.output_bhvr_result = rospy.Publisher("/bhvr_output_res_talking", Bool, queue_size=10)

        self.output_bhvr_command = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)

        # wait for the action server to come up
        while(not self.output_bhvr_command.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            common.loginfo("Waiting for the action server to come up")
        
        common.loginfo("Connection to server done")

    def action_Cb(self, TtsText):
        """
        This method converts text to speech using the proper action server
        made by PAL Robotics.

        Arguments
        ---------
        TtsText : string
            The text the robot has to say.
        """
        goal = pal_interaction_msgs.msg.TtsGoal()
        goal.rawtext.lang_id = TtsText.lang_id
        goal.rawtext.text = TtsText.text

        self.output_bhvr_command.send_goal(goal=goal,done_cb=self.goal_achieve_Cb)

    def goal_achieve_Cb(self):
        self.output_bhvr_result.publish(True)

    def node_shutdown(self):
        self.output_bhvr_command.cancel_goal()
        common.loginfo(self,"have been shutdown")


if __name__ == "__main__":
    """
    It creates a node and starts the tts server in it and connects to all topics needed for this module
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Talking_module()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())

