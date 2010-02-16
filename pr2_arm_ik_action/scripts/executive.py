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

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os
import sys
import time

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from plugs_msgs.msg import *
from std_srvs.srv import *

import actionlib

# Declare list of actions for easy construction
actions = [
    ('tuck_arms',TuckArmsAction),
    ('detect_outlet_action',EmptyAction),
    ('detect_wall_norm',DetectWallNormAction),
    ('detect_plug_on_base_server',DetectPlugOnBaseAction),
    ('move_and_grasp_plug',MoveAndGraspPlugAction),
    ('detect_plug_action',EmptyAction),
    ('plug_wiggle',WigglePlugAction),
    ('stow_plug_action',EmptyAction),
    ('plugin_action',EmptyAction)]

def main():
  rospy.init_node("plugs_executive")

  # reset state of filters
  rospy.wait_for_service('outlet_detector_filter/reset_state')
  rospy.wait_for_service('plug_detector_filter_base/reset_state')
  rospy.wait_for_service('plug_detector_filter_gripper/reset_state')
  reset_filter_srv1 = rospy.ServiceProxy('outlet_detector_filter/reset_state', Empty)
  reset_filter_srv1(EmptyRequest())
  reset_filter_srv2 = rospy.ServiceProxy('plug_detector_filter_base/reset_state', Empty)
  reset_filter_srv2(EmptyRequest())
  reset_filter_srv3 = rospy.ServiceProxy('plug_detector_filter_gripper/reset_state', Empty)
  reset_filter_srv3(EmptyRequest())

  # Construct action ac
  rospy.loginfo("Starting actions.")
  ac = dict()
  for (name,action) in actions:
    ac[name] = actionlib.SimpleActionClient(name,action)

  # Wait for all the ac to start
  for client in ac.values():
    client.wait_for_server()

  rospy.loginfo("All actions started.")

  # Untuck the arms
  untuck_goal = TuckArmsGoal()
  untuck_goal.untuck=True
  untuck_goal.left=True
  untuck_goal.right=True

  untucked = False
  while not untucked:
    rospy.loginfo("Untucking arms...")
    ac['tuck_arms'].send_goal(untuck_goal)
    ac['tuck_arms'].wait_for_result(rospy.Duration(30.0))
    untucked = (GoalStatus.SUCCEEDED == ac['tuck_arms'].get_state())

  empty_goal = EmptyGoal()

  # Detect the wall norm
  rospy.loginfo("Detecting wall norm...")
  ac['detect_wall_norm'].send_goal(empty_goal)
  ac['detect_wall_norm'].wait_for_result(rospy.Duration(30.0))
  if ac['detect_wall_norm'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect wall norm!")
    return

  # Detect the outlet 
  rospy.loginfo("Detecting the outlet ...")
  ac['detect_outlet_action'].send_goal(empty_goal)

  # Detect plug on base
  rospy.loginfo("Detecting the plug on base ...")
  ac['detect_plug_on_base_server'].send_goal(DetectPlugOnBaseGoal())
  ac['detect_plug_on_base_server'].wait_for_result(rospy.Duration(30.0))

  # Finish detecting outlet
  ac['detect_outlet_action'].wait_for_result(rospy.Duration(90.0))
  if ac['detect_outlet_action'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect the outlet!")
    return

  # Finish detecting plug on base
  if ac['detect_plug_on_base_server'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect the plug on the base!")
    return

  # Grasp plug
  rospy.loginfo('Picking up plug...')
  ac['move_and_grasp_plug'].send_goal(MoveAndGraspPlugGoal())
  ac['move_and_grasp_plug'].wait_for_result(rospy.Duration(60.0))

  if ac['move_and_grasp_plug'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to pick up plug!")
    return

  # Move the left arm out of the way
  rospy.loginfo('Clearing left arm...')
  untuck_goal.untuck = True
  untuck_goal.left = True
  untuck_goal.right = False
  ac['tuck_arms'].send_goal(untuck_goal)

  # Detect the plug in gripper
  rospy.loginfo('Detecting plug in gripper...')
  ac['detect_plug_action'].send_goal(empty_goal)
  ac['detect_plug_action'].wait_for_result(rospy.Duration(30.0))
  if ac['detect_plug_action'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect plug in gripper!")
    return

  # Finish moving l arm out of the way
  ac['tuck_arms'].wait_for_result(rospy.Duration(20.0))
  if ac['tuck_arms'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to clear the left arm!")
    return

  # Plug in
  rospy.loginfo('Plugging in...')
  ac['plugin_action'].send_goal(empty_goal)
  ac['plugin_action'].wait_for_result(rospy.Duration(60.0))
  if ac['plugin_action'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to plug in!")
    return

  # Wiggle in
  rospy.loginfo('Wiggling in...')
  wiggle_goal = WigglePlugGoal()
  wiggle_goal.travel.x = 0.05
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  wiggle_goal.abort_threshold = 0.02
  ac['plug_wiggle'].send_goal(wiggle_goal)
  ac['plug_wiggle'].wait_for_result(rospy.Duration(30.0))
  if ac['plug_wiggle'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to wiggle plug in!")
    return

  # Wiggle out
  rospy.loginfo('Wiggling out...')
  wiggle_goal.travel.x = -0.08
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  ac['plug_wiggle'].send_goal(wiggle_goal)
  ac['plug_wiggle'].wait_for_result(rospy.Duration(30.0))

  # Stow plug
  rospy.loginfo('Stowing plug...')
  ac['stow_plug_action'].send_goal(empty_goal)
  ac['stow_plug_action'].wait_for_result(rospy.Duration(45.0))
  if ac['stow_plug_action'].get_state() != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to stow plug!")
    return

  # Tuck the arms
  tuck_goal = TuckArmsGoal()
  tuck_goal.untuck=False
  tuck_goal.left=True
  tuck_goal.right=True

  rospy.loginfo("Tucking arms...")
  ac['tuck_arms'].send_goal(tuck_goal)
  ac['tuck_arms'].wait_for_result(rospy.Duration(30.0))

  # Finish
  rospy.loginfo("Plugged in!")


if __name__ == "__main__":
  main()

