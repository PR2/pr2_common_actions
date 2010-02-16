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
# Author Melonee Wise                                                    


import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import signal

import tf

from pr2_controllers_msgs.msg import *
from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
import dynamic_reconfigure.client
from plugs_stereo_wall.srv import *

prev_handler=None

class DetectWallNormServer:
  def __init__(self, name):
    self.name = name
    self.head_controller = 'head_traj_controller'
    self.head_pub = rospy.Publisher(self.head_controller+'/command', JointTrajectory)
    self.outlet_pub = rospy.Publisher('outlet_frame', PoseWithCovarianceStamped)
    rospy.wait_for_service('plugs_stereo_wall/detect_wall')
    self.detect_wall_srv = rospy.ServiceProxy('plugs_stereo_wall/detect_wall', DetectWall) 
    self.tf_listener = tf.TransformListener()    
    self.projector_client = dynamic_reconfigure.client.Client('camera_synchronizer_node') 
    self.projector_on = {'narrow_stereo_trig_mode': 3}
    self.projector_off = {'narrow_stereo_trig_mode': 4}
    self.server = actionlib.simple_action_server.SimpleActionServer(self.name, DetectWallNormAction)
    self.server.register_goal_callback(self.goalCB)

  def goalCB(self):
    
    # Accept goal
    goal = self.server.accept_new_goal()
    # Create result
    result = DetectWallNormResult()
    # Turn on projector
    self.projector_client.update_configuration(self.projector_on)
    
    head_goal = JointTrajectory()
    head_goal.header.stamp = rospy.Time.now()+rospy.Duration(0.01)
    head_goal.joint_names = ['head_pan_joint', 'head_tilt_joint']
    head_goal.points.append(JointTrajectoryPoint(positions = [0.0, 0.6], velocities = [0.0, 0.0], accelerations = [], time_from_start = rospy.Duration(0.5)))

    self.head_pub.publish(head_goal)
    rospy.sleep(4.0)
    try:
      wall_norm = DetectWallResponse()
      got_points = False
      while(not got_points):
        print "no points yet"
        wall_norm = self.detect_wall_srv(DetectWallRequest())
        if(wall_norm != False):
          got_points =True
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    temp_pose = PoseStamped()
    temp_pose.pose = wall_norm.wall_pose.pose.pose
    temp_pose.header = wall_norm.wall_pose.header 
    temp_transformed_pose =PoseStamped()
    temp_transformed_pose= self.tf_listener.transformPose('base_link',temp_pose)
    wall_norm.wall_pose.header = temp_transformed_pose.header
    wall_norm.wall_pose.pose.pose = temp_transformed_pose.pose
    #fill out covariance 
    wall_norm.wall_pose.pose.covariance = [1E6, 0, 0, 0, 0, 0,
                                 0, 1E6, 0, 0, 0, 0,
                                 0, 0, 1E6, 0, 0, 0,
                                 0, 0, 0, 1E6, 0, 0,
                                 0, 0, 0, 0, 1E-6, 0,
                                 0, 0, 0, 0, 0, 1E-6]

    self.outlet_pub.publish(wall_norm.wall_pose)
    self.projector_client.update_configuration(self.projector_off)
    self.server.set_succeeded(result)



if __name__ == '__main__':
    
  # Initializes a rospy node so that the SimpleActionClient can
  # publish and subscribe over ROS.
  name ='detect_wall_norm'
  rospy.init_node(name)
  detect_wall_norm_server = DetectWallNormServer(name)
  rospy.spin()
