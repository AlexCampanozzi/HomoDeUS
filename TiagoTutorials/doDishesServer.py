#! /usr/bin/env python

import roslib
import rospy
import actionlib

from HomoDeUS.msg import DoDishesAction

class DoDishesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("ok")
    self.server.set_succeeded()


if _name_ == '_main_':
  rospy.init_node('do_dishes_server')
  server = DoDishesServer()
  rospy.spin()