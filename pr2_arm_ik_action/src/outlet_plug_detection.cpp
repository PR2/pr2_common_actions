/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Wim Meeussen
*********************************************************************/


#include <string.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_plugs_msgs/DetectOutletAction.h>
#include <pr2_plugs_msgs/DetectPlugAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>


void outletExecute(const pr2_plugs_msgs::DetectOutletGoalConstPtr& goal);
void plugExecute(const pr2_plugs_msgs::DetectPlugGoalConstPtr& goal);
void outletCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
void plugCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
void publishOutletPose();
void publishPlugPose();
bool gotoJointPosition(double q1, double q2,double q3,double q4,double q5,double q6,double q7);
bool closeGripper();

static actionlib::SimpleActionServer<pr2_plugs_msgs::DetectOutletAction>* outlet_action_server_;
static actionlib::SimpleActionServer<pr2_plugs_msgs::DetectPlugAction>* plug_action_server_;
ros::Publisher outlet_pub_, plug_pub_;
static bool publish_outlet_pose_, publish_plug_pose_;

using namespace actionlib;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plugin");

  ros::NodeHandle node;
  outlet_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("outlet_pose_out", 10);
  plug_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("plug_pose_out", 10);
  outlet_action_server_ = new actionlib::SimpleActionServer<pr2_plugs_msgs::DetectOutletAction>(node,
												"detect_outlet",
												boost::bind(&outletExecute, _1));
  plug_action_server_ = new actionlib::SimpleActionServer<pr2_plugs_msgs::DetectPlugAction>(node,
											"detect_plug",
											boost::bind(&plugExecute, _1));

  ros::spin();
  delete outlet_action_server_;
  delete plug_action_server_;
  return 0;
}


void outletExecute(const pr2_plugs_msgs::DetectOutletGoalConstPtr& goal)
{
  // subscribe to outlet poses
  ros::NodeHandle node;
  ros::Subscriber outlet_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("outlet_pose_in", 10, &outletCallback);

  // close gripper
  if (!closeGripper()){
    ROS_ERROR("Failed to close gripper");
    outlet_action_server_->setAborted();
    return;
  }

  // goto joint space pose
  if (!gotoJointPosition(-1.9972191653524602,-0.12058171791689443,0.71024126541855148,0.67680934702594786,-0.7773483571269979,-1.5696330620286356, -1.7105)){
    ROS_ERROR("Failed to reach joint space position 1");
    outlet_action_server_->setAborted();
    return;
  }

  // goto joint space pose
  if (!gotoJointPosition(-2.7447933658888699,0.14397329806924716,0.90895473979091534,2.4924558651431923,-0.15164934252739068,-1.9417213467203687, -1.5716)){
    ROS_ERROR("Failed to reach joint space position 2");
    outlet_action_server_->setAborted();
    return;
  }

  // publish outlet pose
  publishOutletPose();

  // goto joint space pose
  if (!gotoJointPosition(-1.9972191653524602,-0.12058171791689443,0.71024126541855148,0.67680934702594786,-0.7773483571269979,-1.5696330620286356, -1.7105)){
    ROS_ERROR("Failed to reach joint space position 3");
    outlet_action_server_->setAborted();
    return;
  }

  // goto joint space pose
  if (!gotoJointPosition(-1.8689353042565515,-0.85273005579643579,0.70135879849258354,-0.80974509342955259,-1.9153909193129095,-0.51702175805915029,1.541766134405766)){
    ROS_ERROR("Failed to reach joint space position 4");
    outlet_action_server_->setAborted();
    return;
  }

  // action finished
  pr2_plugs_msgs::DetectOutletResult result;
  outlet_action_server_->setSucceeded(result);
}


void plugExecute(const pr2_plugs_msgs::DetectPlugGoalConstPtr& goal)
{
  // subscribe to plug poses
  ros::NodeHandle node;
  ros::Subscriber plug_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("plug_pose_in", 10, &plugCallback);

  // goto joint space pose
  if (!gotoJointPosition(-1.5368404588445173,-1.0071850839325001,1.1512345995620736,-1.7048130698155768,-1.9479643293742033,-1.1438547699340074,2.3026066201873747)){
    ROS_ERROR("Failed to reach joint space position 1");
    plug_action_server_->setAborted();
    return;
  }

  // goto joint space pose
  if (!gotoJointPosition(-2.0052369066709548,-0.74984294365641635,0.71861616280589224,-3.7275521592158261,-2.0006608683178073,-1.9062180679190992,6.3450203866406909)){
    ROS_ERROR("Failed to reach joint space position 2");
    plug_action_server_->setAborted();
    return;
  }

  // publish plug pose
  publishPlugPose();

  // action finished
  pr2_plugs_msgs::DetectPlugResult result;
  plug_action_server_->setSucceeded(result);
}



bool gotoJointPosition(double q1, double q2,double q3,double q4,double q5,double q6,double q7)
{
  ROS_INFO("Waiting for joint trajectory action");
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> trajectory_action("/r_arm_plugs_controller/joint_trajectory_action", true);
  trajectory_action.waitForServer();
  ROS_INFO("Moving arm to detection pose");

  pr2_controllers_msgs::JointTrajectoryGoal trajectory_goal ;
  trajectory_goal.trajectory.joint_names.resize(7);
  trajectory_goal.trajectory.joint_names[0] = "r_upper_arm_roll_joint";
  trajectory_goal.trajectory.joint_names[1] = "r_shoulder_pan_joint";
  trajectory_goal.trajectory.joint_names[2] = "r_shoulder_lift_joint";
  trajectory_goal.trajectory.joint_names[3] = "r_forearm_roll_joint";
  trajectory_goal.trajectory.joint_names[4] = "r_elbow_flex_joint";
  trajectory_goal.trajectory.joint_names[5] = "r_wrist_flex_joint";
  trajectory_goal.trajectory.joint_names[6] = "r_wrist_roll_joint";

  
  // moves arm to given joint space position
  trajectory_goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);
  trajectory_goal.trajectory.points.resize(1);
  trajectory_goal.trajectory.points[0].positions.resize(7);
  trajectory_goal.trajectory.points[0].positions[0] = q1;
  trajectory_goal.trajectory.points[0].positions[1] = q2;
  trajectory_goal.trajectory.points[0].positions[2] = q3;
  trajectory_goal.trajectory.points[0].positions[3] = q4;
  trajectory_goal.trajectory.points[0].positions[4] = q5;
  trajectory_goal.trajectory.points[0].positions[5] = q6;
  trajectory_goal.trajectory.points[0].positions[6] = q7;
  trajectory_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);
  if (trajectory_action.sendGoalAndWait(trajectory_goal,ros::Duration(20.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Failed to reach joint space detection position");
    return false;
  }
  return true;
}


void publishOutletPose()
{
  ROS_INFO("publishing outlet pose");
  ros::Duration(1.0).sleep();
  // block until pose is published
  publish_outlet_pose_ = true;
  while (ros::ok() && publish_outlet_pose_)
    ros::Duration(0.1).sleep();
  ROS_INFO("published outlet pose");
}

void publishPlugPose()
{
  ROS_INFO("publishing plug pose");
  ros::Duration(1.0).sleep();
  // block until pose is published
  publish_plug_pose_ = true;
  while (ros::ok() && publish_plug_pose_)
    ros::Duration(0.1).sleep();
  ROS_INFO("published plug pose");
}

void outletCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  if (publish_outlet_pose_){
    outlet_pub_.publish(*pose);
    publish_outlet_pose_ = false;
  }
}


void plugCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  if (publish_plug_pose_){
    plug_pub_.publish(*pose);
    publish_plug_pose_ = false;
  }
}


bool closeGripper()
{
  ROS_INFO("Waiting for ripper action");
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_msg;
  gripper_msg.command.position = 0.0;
  gripper_msg.command.max_effort = 100000.0;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_action("r_gripper_controller/gripper_action", true);
  gripper_action.waitForServer();
  ROS_INFO("Closing gripper");
  if (gripper_action.sendGoalAndWait(gripper_msg, ros::Duration(10.0), ros::Duration(5.0)) != SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Gripper failed to close");
    return false;
  }
  return true;
}
