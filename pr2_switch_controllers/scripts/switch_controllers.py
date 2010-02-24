#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_switch_controllers')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_mechanism_msgs.srv import *



def execute_cb(goal):
  rospy.loginfo("Action server received goal")

  rospy.wait_for_service('pr2_controller_manager/switch_controller')
  switch_srv = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
  try:
      switch_srv_success = True
      switch_srv_res = switch_srv(goal.start_controllers, goal.stop_controllers, SwitchControllerRequest.BEST_EFFORT)
  except rospy.ServiceException, e:
      switch_srv_success = False

  if switch_srv_success:
      server.set_succeeded(SwitchControllersResult())
      rospy.loginfo("Controllers switched successfully")  
  else:
      rospy.loginfo("Failed to switch controllers")  
      server.set_aborted()



if __name__ == '__main__':
  #Initialize the node
  name = 'switch_controllers'
  rospy.init_node(name)

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, SwitchControllersAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
