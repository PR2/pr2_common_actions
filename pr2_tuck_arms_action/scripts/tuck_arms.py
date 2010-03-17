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

from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *

import actionlib

# Joint names
joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]

# Tuck trajectory
l_arm_tuck_traj = [[5.0,  0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                   [10.0, 0.01, 1.35,  1.92, -1.68, -1.35, -0.18, 0.31]]
r_arm_tuck_traj = [[5.0, -0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                   [10,  -0.4,  0.0,   0.0,  -2.05,  0.0,  -1.02, 2.51],
                   [15,   0.05, 1.31, -1.38, -2.06, -1.23, -2.02, 3.51]]
r_arm_tuck_single_traj = [[4.0, -0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                          [6,  -0.4,  0.0,   0.0,  -2.05,  0.0,  -1.02, 2.51],
                          [10,   0.05, 1.31, -1.38, -2.06, -1.23, -2.02, 3.51]]


# Untuck trajctory
l_arm_untuck_traj = [[5.0,   0.01, 1.35,  1.92, -1.68, -1.35, -0.18, 0.31],
                     [10.0,  0.01, 1.35,  1.92, -1.68, -1.35, -0.18, 0.31],
                     [15.0,  0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                     [17.0,  0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]

r_arm_untuck_traj = [[5,     0.05, 1.31, -1.38, -2.06, -1.23, -2.02, 3.51],
                     [10,   -0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                     [15,   -0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
                     [17,   -0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]

r_arm_untuck_single_traj = [[2,     0.05, 1.31, -1.38, -2.06, -1.23, -2.02, 3.51],
                            [4,    -0.14, 1.27, -0.90, -1.94, -1.26, -1.97, 3.53],
                            [10,   -0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]

# Clear trajectory
l_arm_clear_traj = [[5.0,  0.4,  1.0,  0.0,  -2.05,  0.0,  -0.1,  0.0]]
r_arm_clear_traj = [[5.0, -0.4,  1.0,  0.0,  -2.05,  0.0,  -0.1,  0.0]]

class TuckArmsActionServer:
        def __init__(self, node_name):
                self.node_name = node_name
                self.tucked = False

                # Get controller name and start joint trajectory action clients
                controller_name = rospy.get_param('~controller_name', 'arm_controller')
                self.left_joint_client = client = actionlib.SimpleActionClient('l_'+controller_name+'/joint_trajectory_action', JointTrajectoryAction)
                self.right_joint_client = client = actionlib.SimpleActionClient('r_'+controller_name+'/joint_trajectory_action', JointTrajectoryAction)

                # Wait for joint clients to connect with timeout
                if not self.left_joint_client.wait_for_server(rospy.Duration(30)):
                        rospy.logerr("pr2_tuck_arms: left_joint_client action server did not come up within timelimit")
                if not self.right_joint_client.wait_for_server(rospy.Duration(30)):
                        rospy.logerr("pr2_tuck_arms: right_joint_client action server did not come up within timelimit")

                # Construct action server
                self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmsAction, self.executeCB)

        def executeCB(self, goal):
                # Create a new result
                result = TuckArmsResult()
                result.left = True
                result.right = True

                if not goal.untuck:
                        if goal.right and not goal.left:
                                rospy.loginfo("Tucking only right arm")
                                self.go('r', r_arm_tuck_single_traj)

                        if goal.right and goal.left:
                                rospy.loginfo("Tucking right arm")
                                self.go('r', r_arm_tuck_traj)
                        if goal.left:
                                rospy.loginfo("Tucking left arm")
                                self.go('l', l_arm_tuck_traj)
                else:
                        if self.tucked:

                                if goal.right and not goal.left:
                                        rospy.loginfo("Untucking only right arm")
                                        self.go('r', r_arm_untuck_single_traj)

                                if goal.right and goal.left:
                                        rospy.loginfo("Untucking right arm")
                                        self.go('r', r_arm_untuck_traj)

                                if goal.left:
                                        rospy.loginfo("Untucking left arm")
                                        self.go('l', l_arm_untuck_traj)
                        else:
                                if goal.right or goal.left:
                                        rospy.loginfo("Clearing right arm")
                                        self.go('r', r_arm_clear_traj)

                                if goal.left:
                                        rospy.loginfo("Clearing left arm")
                                        self.go('l', l_arm_clear_traj)


                # Wait for results
                if goal.right:
                        right_finished = self.right_joint_client.wait_for_result()
                        result.right = (GoalStatus.SUCCEEDED == self.right_joint_client.get_state())
                if goal.left:
                        left_finishehd = self.left_joint_client.wait_for_result()
                        result.left = (GoalStatus.SUCCEEDED == self.left_joint_client.get_state())

                # Succeed or fail
                if not goal.untuck or result.left and result.right:
                        self.action_server.set_succeeded(result)
                        if goal.untuck:
                                rospy.loginfo("Untuck arms SUCCEEDED")
                                self.tucked=False
                        else:
                                rospy.loginfo("Tuck arms SUCCEEDED")
                                self.tucked=True
                else:
                        rospy.logerr("Tuck or untuck arms FAILED")
                        self.action_server.set_aborted(result)

        def go(self, side, positions):
                goal = JointTrajectoryGoal()
                goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
                goal.trajectory.points = []

                for p in positions:
                        goal.trajectory.points.append(JointTrajectoryPoint( positions = p[1:],
                                                                            velocities = [0.0] * (len(p) - 1),
                                                                            accelerations = [],
                                                                            time_from_start = rospy.Duration(p[0])))

                goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

                {'l': self.left_joint_client, 'r': self.right_joint_client}[side].send_goal(goal)

if __name__ == '__main__':
        action_name = 'tuck_arms'
        rospy.init_node(action_name)

        tuck_arms_action_server = TuckArmsActionServer(action_name)

        rospy.spin()


