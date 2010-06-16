#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Modified by Kevin Watts for two arm use
# Modified by Jonathan Bohren to be an action and for untucking

import roslib
import signal
roslib.load_manifest('pr2_tuck_arms_action')

import rospy

import os
import sys
import time
import math

from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
import getopt
import actionlib

# Joint names
joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]


l_arm_tucked = [0.023, 1.290, 1.891, -1.686, -1.351, -0.184, 0.307]
r_arm_tucked = [0.039, 1.262, -1.366, -2.067, -1.231, -1.998, 0.369]
l_arm_untucked = [ 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_untucked = [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_approach = [0.039, 1.262, 0.0, -2.067, -1.231, -1.998, 0.369]
r_arm_up_traj = [[-0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]

# Tuck trajectory
l_arm_tuck_traj = [l_arm_tucked]
r_arm_tuck_traj = [r_arm_approach,
                   r_arm_tucked]

# Untuck trajctory
l_arm_untuck_traj = [l_arm_untucked]
r_arm_untuck_traj = [r_arm_approach,
                     r_arm_untucked]

# clear trajectory
l_arm_clear_traj = [l_arm_untucked]
r_arm_clear_traj = [r_arm_untucked]

quit_when_finished = False

class TuckArmsActionServer:
  def __init__(self, node_name):
    self.node_name = node_name

    # arm state: -1 unknown, 0 tucked, 1 untucked
    self.l_arm_state = -1
    self.r_arm_state = -1
    self.success = True

    # Get controller name and start joint trajectory action clients
    self.move_duration = rospy.get_param('~move_duration', 2.5)
    action_name = rospy.get_param('~joint_trajectory_action', 'joint_trajectory_action')
    self.left_joint_client = client = actionlib.SimpleActionClient('l_arm_controller/'+action_name, JointTrajectoryAction)
    self.right_joint_client = client = actionlib.SimpleActionClient('r_arm_controller/'+action_name, JointTrajectoryAction)


    # Connect to controller state
    rospy.Subscriber('l_arm_controller/state', JointTrajectoryControllerState ,self.stateCb)
    rospy.Subscriber('r_arm_controller/state', JointTrajectoryControllerState ,self.stateCb)

    # Wait for joint clients to connect with timeout
    if not self.left_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("pr2_tuck_arms: left_joint_client action server did not come up within timelimit")
    if not self.right_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("pr2_tuck_arms: right_joint_client action server did not come up within timelimit")

    # Construct action server
    self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmsAction, self.executeCB)


  def executeCB(self, goal):
    # Make sure we received arm state
    while not self.r_received or not self.l_received:
      rospy.sleep(0.1)

    # Create a new result
    result = TuckArmsResult()
    result.left = True
    result.right = True

    # Tucking
    if not goal.untuck:
      # tuck both arms
      if goal.right and goal.left:
        rospy.loginfo('Tucking left arm and right arm...')
        # left arm is tucked already
        if self.l_arm_state == 0:
          # right arm is tucked already
          if self.r_arm_state == 0:
            rospy.loginfo('  ...both arm are tucked already')
          # right arm is not tucked yet
          else:
            rospy.loginfo('  ...tucking right arm')
            self.untuckR()
            self.tuckR()
        # left arm is not tucked 
        else:
          rospy.loginfo('  ...untucking and tucking both arms')
          self.untuckL()
          self.tuckL()
          self.tuckR()


      # tuck left arm only
      elif goal.left:
        rospy.loginfo('Tucking left arm...')
        # left arm is tucked already
        if self.l_arm_state == 0:
          rospy.loginfo('  ...left arm is already tucked')          
        # right arm is tucked
        elif self.r_arm_state == 0:
          rospy.loginfo('  ...untucking right arm and tucking both arms')             
          self.untuckR()
          self.tuckL()
          self.tuckR()
        # right arm is not tucked
        else:
          rospy.loginfo('  ...untucking right arm and tucking left')          
          self.untuckR()
          self.tuckL()


      # tuck right arm only
      elif goal.right:
        rospy.loginfo('Tucking right arm...')        
        # right arm is tucked already
        if self.r_arm_state == 0:
          rospy.loginfo('  ...right arm is already tucked')
        # left arm is tucked
        elif self.l_arm_state == 0:
          rospy.loginfo('  ...tucking right arm')          
          self.tuckR()
        # left arm is not tucked
        else:
          rospy.loginfo('  ...untucking left arm and tucking right arm')          
          self.untuckL()
          self.tuckR()

    # UnTucking
    else:
      if goal.right and not goal.left:
        rospy.loginfo("Untucking only right arm...")
        rospy.loginfo("  ...untucking right arm")
        self.untuckR()
      elif goal.right and goal.left:
        rospy.loginfo("Untucking right and left arms...")
        rospy.loginfo("  ...untucking boht arms")
        self.untuckR()
        self.untuckL()
        self.untuckR()
      elif goal.left:
        rospy.loginfo("Untucking left arm...")
        if self.r_arm_state == 0:
          rospy.loginfo("  ...untucking both arms, tucking right arm")          
          self.untuckR()
          self.untuckL()
          self.tuckR()
        else:
          rospy.loginfo("  ...untucking both arms")                    
          self.untuckR()
          self.untuckL()
          self.untuckR()

    # Succeed or fail
    if self.success:
      self.action_server.set_succeeded(result)
      if goal.untuck:
        rospy.loginfo("Untuck arms SUCCEEDED")
      else:
        rospy.loginfo("Tuck arms SUCCEEDED")
    else:
      rospy.logerr("Tuck or untuck arms FAILED: %d %d"%(result.left, result.right))
      self.action_server.set_aborted(result)


  # clears r arm
  def tuckL(self):
    if self.l_arm_state != 0:
      self.go('r', r_arm_up_traj)
      if self.l_arm_state != 1:
        self.go('l', l_arm_clear_traj)
      self.go('l', l_arm_tuck_traj)
      self.go('r', r_arm_clear_traj)
    
  # clears r arm
  def untuckL(self):
    if self.l_arm_state != 1:
      self.go('r', r_arm_up_traj)
      if self.l_arm_state == 0:
        self.go('l', l_arm_untuck_traj)
      elif self.l_arm_state == -1:
        self.go('l', l_arm_clear_traj)

  # assumes l tucked or cleared
  def tuckR(self):
    if self.r_arm_state != 0:
      self.go('r', r_arm_tuck_traj)

  # assumes l tucked or cleared
  def untuckR(self):
    if self.r_arm_state == 0:
      self.go('r', r_arm_untuck_traj)
    elif self.r_arm_state == -1:
      self.go('r', r_arm_clear_traj)

  def go(self, side, positions, wait = True):
    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
    goal.trajectory.points = []
    for p, count in zip(positions, range(0,len(positions)+1)):
      goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                          velocities = [],
                                                          accelerations = [],
                                                          time_from_start = rospy.Duration((count+1) * self.move_duration)))
    goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
    if wait:
      if not {'l': self.left_joint_client, 'r': self.right_joint_client}[side].send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0)):
        self.success = False
    else:
      {'l': self.left_joint_client, 'r': self.right_joint_client}[side].send_goal(goal)


  def stateCb(self, msg):
    l_sum_tucked = 0
    l_sum_untucked = 0
    r_sum_tucked = 0
    r_sum_untucked = 0
    for name_state, name_desi, value_state, value_l_tucked, value_l_untucked, value_r_tucked, value_r_untucked in zip(msg.joint_names, joint_names, msg.actual.positions , l_arm_tucked, l_arm_untucked, r_arm_tucked, r_arm_untucked):
      if 'l_'+name_desi+'_joint' == name_state:
        self.l_received = True
        l_sum_tucked = l_sum_tucked + math.fabs(value_state - value_l_tucked)
        l_sum_untucked = l_sum_untucked + math.fabs(value_state - value_l_untucked)
      if 'r_'+name_desi+'_joint' == name_state:
        self.r_received = True
        r_sum_tucked = r_sum_tucked + math.fabs(value_state - value_r_tucked)
        r_sum_untucked = r_sum_untucked + math.fabs(value_state - value_r_untucked)

    if l_sum_tucked > 0 and l_sum_tucked < 0.1:
      self.l_arm_state = 0
    elif l_sum_untucked > 0 and l_sum_untucked < 0.3:
      self.l_arm_state = 1
    elif l_sum_tucked >= 0.1 and l_sum_untucked >= 0.3:
      self.l_arm_state = -1    

    if r_sum_tucked > 0 and r_sum_tucked < 0.1:
      self.r_arm_state = 0
    elif r_sum_untucked > 0 and r_sum_untucked < 0.3:
      self.r_arm_state = 1
    elif r_sum_tucked >= 0.1 and r_sum_untucked >= 0.3:
      self.r_arm_state = -1    


if __name__ == '__main__':
  action_name = 'tuck_arms'
  rospy.init_node(action_name)
  tuck_arms_action_server = TuckArmsActionServer(action_name)

  # check for command line arguments, and send goal to action server if required
  myargs = rospy.myargv()[1:]
  if len(myargs):
    goal = TuckArmsGoal()
    goal.left = False
    goal.right = False
    opts, args = getopt.getopt(myargs, 'ql:r:', ['quit','left','right'])
    for arm, action in opts:
      
      if arm in ('-l', '--left'):
        goal.left = True
        if action in ('tuck', 't'):
          if goal.right and goal.untuck:
            rospy.logerr('Cannot tuck left arm while untucking right arm')
            exit()
          goal.untuck = False
        elif action in ('untuck', 'u'):
          if goal.right and not goal.untuck:
            rospy.logerr('Cannot untuck left arm while tucking right arm')
            exit()
          goal.untuck = True
        else:
          rospy.logerr('Unknown option for left arm: %s. Use "tuck" or "untuck"'%action)
          exit()

      if arm in ('-r', '--right'):
        goal.right = True
        if action in ('tuck', 't'):
          if goal.left and goal.untuck:
            rospy.logerr('Cannot tuck right arm while untucking left arm')
            exit()
          goal.untuck = False
        elif action in ('untuck', 'u'):
          if goal.left and not goal.untuck:
            rospy.logerr('Cannot untuck right arm while tucking left arm')
            exit()
          goal.untuck = True
        else:
          rospy.logerr('Unknown option for right arm: %s. Use "tuck" or "untuck"'%action)
          exit()
      if arm in ('--quit', '-q'):
        quit_when_finished = True
    
    tuck_arm_client = actionlib.SimpleActionClient(action_name, TuckArmsAction)
    rospy.logdebug('Waiting for action server to start')
    tuck_arm_client.wait_for_server(rospy.Duration(10.0))
    rospy.logdebug('Sending goal to action server')
    tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    if quit_when_finished:
      exit()
    rospy.spin()
  else:
    rospy.spin()

