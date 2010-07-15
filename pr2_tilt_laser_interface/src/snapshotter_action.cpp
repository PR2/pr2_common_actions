/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

// Action Interface
#include <actionlib/server/action_server.h>
#include <pr2_tilt_laser_interface/GetSnapshotAction.h>

// Controller Interface
#include <pr2_msgs/LaserScannerSignal.h>

using namespace pr2_tilt_laser_interface;

namespace SnapshotStates
{
  enum SnapshotState
  {
    WAITING_FOR_START_SIGNAL = 0,
    WAITING_FOR_END_SIGNAL = 1,
    IDLE = 2
  };
}
typedef SnapshotStates::SnapshotState SnapshotState;

typedef actionlib::ActionServer<pr2_tilt_laser_interface::GetSnapshotAction> SnapshotActionServer;

class Snapshotter
{
public:
  Snapshotter();

  // Action Interface
  void goalCallback(SnapshotActionServer::GoalHandle gh);
  void cancelCallback(SnapshotActionServer::GoalHandle gh);

private:
  ros::NodeHandle nh_;
  SnapshotActionServer as_;

  boost::mutex state_mutex_;
  SnapshotState state_;

  SnapshotActionServer::GoalHandle current_gh_;
};

Snapshotter::Snapshotter() :
  as_(nh_, "get_laser_snapshot"),
  state_(SnapshotStates::IDLE)
{
  as_.registerGoalCallback(    boost::bind(&Snapshotter::goalCallback,    this, _1) );
  as_.registerCancelCallback(  boost::bind(&Snapshotter::cancelCallback, this, _1) );

  // Todo: Subscribe to Laser Scans
  // Todo: Subscribe to Laser Signals
}

void Snapshotter::goalCallback(SnapshotActionServer::GoalHandle gh)
{
  // Check validity of the new goal
  if (gh.getGoal()->command_laser)
  {
    ROS_DEBUG("Rejecting Goal.  We don't know how to command the laser yet");
    gh.setRejected();
    return;
  }

  {
    boost::mutex::scoped_lock lock(state_mutex_);

    // Preemption Logic
    if (state_ != SnapshotStates::IDLE)
    {
      current_gh_.setCanceled();
      state_ = SnapshotStates::IDLE;
    }

    // Load the new goal
    assert(state_ == SnapshotStates::IDLE);
    current_gh_ = gh;
    state_ = SnapshotStates::WAITING_FOR_START_SIGNAL;
  }
}

void Snapshotter::cancelCallback(SnapshotActionServer::GoalHandle gh)
{
  boost::mutex::scoped_lock lock(state_mutex_);

  // See if our current goal is the one that needs to be cancelled
  if (current_gh_ == gh)
  {
    current_gh_.setCanceled();
    state_ = SnapshotStates::IDLE;
  }
  else
    ROS_DEBUG("Got a cancel request for some other goal. Ignoring it");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_snapshotter");
  Snapshotter snapshotter;
  ros::spin();
}
