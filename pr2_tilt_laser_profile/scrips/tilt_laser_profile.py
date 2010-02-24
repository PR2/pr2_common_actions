#!/usr/bin/env python

PKG = "pr2_tilt_laser_profile"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string
import dynamic_reconfigure.client

import rospy
import actionlib
from std_msgs import *

from pr2_msgs.msg import LaserTrajCmd
from actionlib_msgs.msg import *
from pr2_msgs.srv import *
from pr2_common_action_msgs.msg import *
from time import sleep

old_config = {}

class laserTiltProfile:
    def __init__(self, name, laser_controller_name, laser_node_name):
        self.laser_controller_name = laser_controller_name
        self.laser_node_name = laser_node_name


        # create action server
        self.server = actionlib.simple_action_server.SimpleActionServer(name, LaserTiltProfileAction, self.execute_cb)


    def on_shutdown(self):
        self.restore_laser(self.laser_node_name)
        self.set_tilt_profile(self.laser_controller_name, [0.0, 0.0], [0.0, 1.0])

    def set_tilt_profile(self, controller, position, time_from_start):
        cmd = LaserTrajCmd()
        cmd.header   =    roslib.msg.Header(None, None, None)
        cmd.profile  = "blended_linear"

        if len(position) != len(time_from_start):
            rospy.logerr('Cannot set this profile. Length of position not equal to length of time waypoints. Posititions: %s, time waypoints: %s' % (position, time))
            return
    
        cmd.position = position
        cmd.time_from_start = time_from_start
        cmd.time_from_start = [rospy.Time.from_sec(x) for x in cmd.time_from_start]
        cmd.max_velocity     = 10
        cmd.max_acceleration = 30

        print 'Sending Command to %s: ' % controller
        print '  Profile Type: %s' % cmd.profile
        print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.position])
        print '  Time: %s' % ','.join(['%.3f' % x.to_sec() for x in cmd.time_from_start])
        print '  MaxRate: %f' % cmd.max_velocity
        print '  MaxAccel: %f' % cmd.max_acceleration

        rospy.wait_for_service(controller + '/set_traj_cmd')                                        
        s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
        resp = s.call(SetLaserTrajCmdRequest(cmd))
        
        print 'Command sent!'
        print '  Resposne: %f' % resp.start_time.to_sec()

    def claim_laser(self, tilt_laser_node):
        global old_config

        #TODO: remove hack to get things working in gazebo
        try: 
            rospy.wait_for_service(tilt_laser_node + '/set_parameters', 1.0)
        except rospy.exceptions.ROSException, e:
            return
        #end TODO

        client = dynamic_reconfigure.client.Client(tilt_laser_node)
        old_config = client.get_configuration(2.0)
        new_config = {'skip': 0, 'intensity': 0, 'min_ang': -1.57, 'max_ang': 1.57, 'calibrate_time': 1, 'cluster': 1, 'time_offset': 0.0} 
        rospy.loginfo('Setting laser to the navigation configuration: %s' % new_config)
        client.update_configuration(new_config)

    def restore_laser(self, tilt_laser_node):
        #TODO: remove hack to get things working in gazebo
        try: 
            rospy.wait_for_service(tilt_laser_node + '/set_parameters', 1.0)
        except rospy.exceptions.ROSException, e:
            return
        #end TODO

        rospy.loginfo('Setting laser back to this configuration: %s' % old_config)
        client = dynamic_reconfigure.client.Client(tilt_laser_node)
        client.update_configuration(old_config)

    def execute_cb(self, goal):
        rospy.loginfo("Action server received goal")
        self.claim_laser(self.laser_node_name)
        self.set_tilt_profile(self.laser_controller_name, [1.05,  -.7, 1.05], [0.0, 1.8, 2.0125 + .3])
        self.server.set_succeeded(SwitchControllersResult())

def print_usage(exit_code = 0):
    print '''Usage:
    tilt_laser_profile.py [controller] [laser_node_name]
'''
    sys.exit(exit_code)



if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_usage()

    name = 'nav_tilt_profile_manager'
    rospy.init_node(name)

    laser_profile = laserTiltProfile(name, sys.argv[1], sys.argv[2])
    laser_profile.execute_cb(LaserTiltProfileGoal())
    rospy.on_shutdown(laser_profile.on_shutdown)
    rospy.spin()


