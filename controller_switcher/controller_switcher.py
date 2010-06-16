#! /usr/bin/env python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('controller_switcher')
import rospy
from pr2_mechanism_msgs.srv import *
from actionlib.action_server import ActionServer
from controller_switcher.msg import *

class ControllerSwitcher:
    def __init__(self):
        self.groups = rospy.get_param('controller_switcher/groups')

        self.server = ActionServer('action', RequestControllersAction, self.goal_cb)
        self.list_controllers = \
            rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
        self.switch_controllers = \
            rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

    def controllers_up(self):
        resp = self.list_controllers()
        up = []
        for i in range(len(resp.controllers)):
            if resp.state[i] == 'running':
                up.append(resp.controllers[i])
        return up
        
    def goal_cb(self, gh):
        gh.set_accepted()
        to_start = gh.get_goal().controllers
        to_stop = []

        up = self.controllers_up()
        print "UP:", up

        for group in self.groups.values():
            for c in to_start:
                if c in group:
                    for g in group:
                        if g not in to_start and g in up:
                            to_stop.append(g)
                    break

        for c in up:
            if c in to_start:
                to_start.remove(c)

        print "Bringing up:", to_start
        print "Taking down:", to_stop

        resp = self.switch_controllers(
            SwitchControllerRequest(start_controllers = to_start,
                                    stop_controllers = to_stop,
                                    strictness = SwitchControllerRequest.STRICT))
        if resp.ok:
            gh.set_succeeded()
        else:
            gh.set_aborted()

def main():
    rospy.init_node('controller_switcher')
    cs = ControllerSwitcher()
    rospy.spin()

if __name__ == '__main__': main()
