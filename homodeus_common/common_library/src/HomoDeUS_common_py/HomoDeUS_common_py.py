"""
        The purpose of this common python module is to gather every functions created my the team members 
        that can be use in different contexts so other team members can use it from anywhere in a near future.

        To use it, only type :
        import HomoDeUS_common_py.HomoDeUS_common_py as common  
"""
import os
import re
import rospy
import unicodedata

from ctypes import *
from hbba_msgs.msg import Desire
from contextlib import contextmanager
from geometry_msgs.msg import Pose
from tf_lookup.srv import lookupTransform
from tf.transformations import euler_from_quaternion

try:
    import httplib
except:
    import http.client as httplib


def convert_char_array_to_string(char_array):
    """
        This function convert a cstring,also called char_array, into a string.
        It returns the string representing the cstring or an error message into a string

        Arguments
        ---------
        char_array: cstring or an array of chararcter 
            such as 'h','e','l','l','o',' ', 'w','o','r','l','d'
    """
    str = ""
    try:
        return str.join(char_array)
    except:
        return "Error occured: variable non string"


def convert_pose_quaternion_to_euler(pose):
    pose.orientation = list(euler_from_quaternion((pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w)))
    return pose
    
def get_file_name(file):
    """
        This function convert an absolute path of a path into the name of the file it points without the extension
        It returns the name of file without the extension or the same string it receives if it is not a path
        
        Arguments
        ---------
        file: string
            the absolute path usually get from __file__
    """
    return os.path.splitext(os.path.basename(file))[0]


def equalWithinTolerance(a, b, tol):
    """
        This function check if two values are equal within a given tolerance.
        when it is equal, it returns True. Otherwise it return False
        
        Arguments
        ---------
        a: int or float
            The first value
        b: int or float
            The second value
        tol: int or float
            the tolerance within which the difference between the two value is considered non relevant.
    """
    return abs(a - b) <= tol


def no_caps_and_ponctuation(text):
    """
        This function return the text received without whitespace and Caps
        
        Arguments
        ---------
        text: string
            The text to alter
    """
    return re.sub(r'[^\w\s]', '', text).lower()

def add_desire(origin,desire_id, desire_type,desire_utility,desire_intensity, desire_params=None):
        """
        This method adds a desire to the iw 
        
        Arguments
        ---------
        desire_id: string
            The name of the desire
        desire_type: string
            the type of the desire, should fit with one listed in desires.txt in homodeus_hbba_cfg
        desire_utility: float
            The utility of the desire so hbba knows which strategy to use
        desire_intensity: float
            The intensity reprensenting the importance of the desire to be done
        desire_params: string
            other important informations used by the strategy to fulfill the desire
        """
        des = Desire()
        des.id          = desire_id
        des.type        = desire_type
        des.utility     = desire_utility
        des.intensity   = desire_intensity
        if desire_params is not None:
            des.params = desire_params
        
        rospy.loginfo("adding desire: " + des.id)

        origin.add_desires_service.call([des])

def loginfo(origin, text=""):
    """
        This function allow a better way to debug code by logging from which class the rospy.logxxxx was called.
        When it is not called from a class, it can also display from which file it comes from.
        Otherwise, it can also work as a normal rospy.log when not enough information are given.
        
        Arguments
        ---------
        origin: InstanceOfAnyClass, string (usually)
            should be the value telling from where rospy.logxxx was called
        text: string
            the information to log
    """
    try:
        if not isinstance(origin, (basestring, int, float, list)):
            # should be a class instance
            rospy.loginfo(origin.__class__.__name__ + ": " + text)
        else:
            # should usually be the file name
            rospy.loginfo(get_file_name(str(origin)) + ": " + text)
    except Exception:
        rospy.logfatal("There was a problem in the common library")


def logerr(origin, text=""):
    """
        This function allow a better way to debug code by logging from which class the rospy.logxxxx was called.
        When it is not called from a class, it can also display from which file it comes from.
        Otherwise, it can also work as a normal rospy.log when not enough information are given.
        
        Arguments
        ---------
        origin: InstanceOfAnyClass, string (usually)
            should be the value telling from where rospy.logxxx was called
        text: string
            the information to log
    """
    try:
        if not isinstance(origin, (basestring, int, float, list)):
            # should be a class instance
            rospy.logerr(origin.__class__.__name__ + ": " + text)
        else:
            # should usually be the file name
            rospy.logerr(get_file_name(str(origin)) + ": " + text)
    except Exception:
        rospy.logfatal("There was a problem in the common library")


def logdebug(origin, text=""):
    """
        This function allow a better way to debug code by logging from which class the rospy.logxxxx was called.
        When it is not called from a class, it can also display from which file it comes from.
        Otherwise, it can also work as a normal rospy.log when not enough information are given.
        
        Arguments
        ---------
        origin: InstanceOfAnyClass, string (usually)
            should be the value telling from where rospy.logxxx was called
        text: string
            the information to log
    """
    try:
        if not isinstance(origin, (basestring, int, float, list)):
            # should be a class instance
            rospy.logdebug(origin.__class__.__name__ + ": " + text)
        else:
            # should usually be the file name
            rospy.logdebug(get_file_name(str(origin)) + ": " + text)
    except Exception:
        rospy.logfatal("There was a problem in the common library")


def logwarn(origin, text=""):
    """
        This function allow a better way to debug code by logging from which class the rospy.logxxxx was called.
        When it is not called from a class, it can also display from which file it comes from.
        Otherwise, it can also work as a normal rospy.log when not enough information are given.
        
        Arguments
        ---------
        origin: InstanceOfAnyClass, string (usually)
            should be the value telling from where rospy.logxxx was called
        text: string
            the information to log
    """
    try:
        if not isinstance(origin, (basestring, int, float, list)):
            # should be a class instance
            rospy.logwarn(origin.__class__.__name__ + ": " + text)
        else:
            # should usually be the file name
            rospy.logwarn(get_file_name(str(origin)) + ": " + text)
    except Exception:
        rospy.logfatal("There was a problem in the common library")


def logfatal(origin, text=""):
    """
        This function allow a better way to debug code by logging from which class the rospy.logxxxx was called.
        When it is not called from a class, it can also display from which file it comes from.
        Otherwise, it can also work as a normal rospy.log when not enough information are given.
        
        Arguments
        ---------
        origin: InstanceOfAnyClass, string (usually)
            should be the value telling from where rospy.logxxx was called
        text: string
            the information to log
    """
    try:
        if not isinstance(origin, (basestring, int, float, list)):
            # should be a class instance
            rospy.logfatal(origin.__class__.__name__ + ": " + text)
        else:
            # should usually be the file name
            rospy.logfatal(get_file_name(str(origin)) + ": " + text)
    except Exception:
        rospy.logfatal("There was a problem in the common library")

def get_relative_pose(base_frame, moved_frame):
    rospy.wait_for_service('/lookupTransform')
    lookup = rospy.ServiceProxy("/lookupTransform", lookupTransform)
    try:
        response = lookup(base_frame, moved_frame, rospy.Time())
    except rospy.ServiceException as exc:
        rospy.logwarn("lookup did not process request: " + str(exc))
        raise rospy.ServiceException
    
    pose = Pose()
    pose.position = response.transform.transform.translation
    pose.orientation = response.transform.transform.rotation
    return pose


class PID:
    def __init__(self, ref, K_P=1.0, K_I=1.0, K_D=1.0):
        self.ref = ref

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

        self.sum = 0.

        self.last_timestamp = rospy.get_time()

        self.last_error = 0.

    def get_next_command(self, pos, epsilon=1e-20):
        err = pos - self.ref

        now = rospy.get_time()
        dT = now - self.last_timestamp

        # Proportial term
        P = self.K_P * err

        # Integration term
        self.sum += err * dT

        I = self.K_I * self.sum

        # Derivative term
        dErr = (err - self.last_error) / (dT + epsilon)

        D = self.K_D * dErr

        cmd = P + I + D

        self.last_timestamp = now

        return cmd

    def set_coefficients(self,K_P, K_I, K_D):
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D


# To hide ALSA Lib error message so it doesn't polute our log
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)


def py_error_handler(filename, line, function, err, fmt):
    pass


c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)


@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)


def check_connection():
    conn = httplib.HTTPConnection("www.google.com", timeout=1)
    try:
        conn.request("HEAD", "/")
        conn.close()
        return True
    except:
        conn.close()
        return False

def remove_accents(input_str):
    unicode_string = input_str.decode("utf-8")
    nfkd_form = unicodedata.normalize('NFKD', unicode_string)
    only_ascii = nfkd_form.encode('ASCII', 'ignore')
    return only_ascii