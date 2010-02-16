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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_plugs_msgs/StowPlugAction.h>
#include <pr2_plugs_msgs/PR2ArmIKAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

static std::string plug_base_frame, plug_gripper_frame, gripper_frame, base_frame;
static actionlib::SimpleActionServer<pr2_plugs_msgs::StowPlugAction>* action_server_;
static pr2_plugs_msgs::PR2ArmIKGoal ik_goal_;

void planGripperTrajectory(tf::Transformer& tf);
bool getGripperPose(tf::Transformer& tf, const ros::Time& time, geometry_msgs::PoseWithCovarianceStamped& pose);
void execute(const pr2_plugs_msgs::StowPlugGoalConstPtr& goal);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plugin");

  ros::NodeHandle node_private("~");
  node_private.param("plug_base_frame", plug_base_frame, std::string("plug_base_frame_not_specified"));
  node_private.param("plug_gripper_frame", plug_gripper_frame, std::string("plug_gripper_frame_not_specified"));
  node_private.param("gripper_frame", gripper_frame, std::string("gripper_frame_not_specified"));
  node_private.param("base_frame", base_frame, std::string("base_frame_not_specified"));
  double shoulder_pan_,shoulder_lift_,upper_arm_roll_,elbow_flex_,forearm_roll_,wrist_flex_,wrist_roll_;
  node_private.getParam("pose_suggestion/shoulder_pan", shoulder_pan_ );
  node_private.getParam("pose_suggestion/shoulder_lift", shoulder_lift_ );
  node_private.getParam("pose_suggestion/upper_arm_roll", upper_arm_roll_);
  node_private.getParam("pose_suggestion/elbow_flex", elbow_flex_);
  node_private.getParam("pose_suggestion/forearm_roll", forearm_roll_);
  node_private.getParam("pose_suggestion/wrist_flex",  wrist_flex_);
  node_private.getParam("pose_suggestion/wrist_roll",  wrist_roll_);
    
  ik_goal_.ik_seed.name.push_back("r_shoulder_pan_joint");
  ik_goal_.ik_seed.name.push_back("r_shoulder_lift_joint");
  ik_goal_.ik_seed.name.push_back("r_upper_arm_roll_joint");
  ik_goal_.ik_seed.name.push_back("r_elbow_flex_joint");
  ik_goal_.ik_seed.name.push_back("r_forearm_roll_joint");
  ik_goal_.ik_seed.name.push_back("r_wrist_flex_joint");
  ik_goal_.ik_seed.name.push_back("r_wrist_roll_joint");
  ik_goal_.ik_seed.position.push_back(shoulder_pan_);
  ik_goal_.ik_seed.position.push_back(shoulder_lift_);
  ik_goal_.ik_seed.position.push_back(upper_arm_roll_);
  ik_goal_.ik_seed.position.push_back(elbow_flex_);
  ik_goal_.ik_seed.position.push_back(forearm_roll_);
  ik_goal_.ik_seed.position.push_back(wrist_flex_);
  ik_goal_.ik_seed.position.push_back(wrist_roll_);
    
  action_server_ = new actionlib::SimpleActionServer<pr2_plugs_msgs::StowPlugAction>(ros::NodeHandle(),
										     "stow_plug",
										     boost::bind(&execute, _1));
  ros::spin();
  delete action_server_;
  return 0;
}



void execute(const pr2_plugs_msgs::StowPlugGoalConstPtr& goal)
{
  ros::NodeHandle node;
  ros::Publisher pub_gripper = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("gripper_topic", 10);
  tf::TransformListener tf;

  // create action for ik
  ROS_INFO("creating action client for ik");
  actionlib::SimpleActionClient<pr2_plugs_msgs::PR2ArmIKAction> move_cart_action("arm_ik", true);
  move_cart_action.waitForServer();

  // get pose from plug_gripper to gripper and from base to plug_base
  ROS_INFO("Get poses from tf");
  tf::StampedTransform pose_plug_gripper, pose_base_plug;
  ros::Time time = ros::Time::now();
  if (!tf.waitForTransform(plug_gripper_frame, gripper_frame, time, ros::Duration(3.0))){
    ROS_ERROR("Could not transform from %s to %s at time %f", gripper_frame.c_str(), plug_gripper_frame.c_str(), time.toSec());
    action_server_->setAborted();
    return;
  }
  tf.lookupTransform(plug_gripper_frame, gripper_frame, time, pose_plug_gripper);
  if (!tf.waitForTransform(base_frame, plug_base_frame, time, ros::Duration(3.0))){
    ROS_ERROR("Could not transform from %s to %s at time %f", base_frame.c_str(),plug_gripper_frame.c_str(), time.toSec());
    action_server_->setAborted();
    return;
  }
  tf.lookupTransform(base_frame, plug_base_frame, time, pose_base_plug);

  // move plug above stowing pose
  ROS_INFO("Move plug above stowing pose");
  tf::Transform pose_plug_plug(tf::Quaternion::getIdentity(), tf::Vector3(0.0, -0.06, 0.0));
  tf::poseTFToMsg(pose_base_plug*pose_plug_plug*pose_plug_gripper, ik_goal_.pose.pose);
  ik_goal_.pose.header.stamp = ros::Time::now();
  ik_goal_.pose.header.frame_id = base_frame;
  if (move_cart_action.sendGoalAndWait(ik_goal_, ros::Duration(60.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Failed to reach pose above stowing pose");
    action_server_->setAborted();
    return;
  }

  // move plug into stowing pose
  ROS_INFO("Move plug into stowing pose");
  tf::poseTFToMsg(pose_base_plug*pose_plug_gripper, ik_goal_.pose.pose);
  ik_goal_.pose.header.stamp = ros::Time::now();
  ik_goal_.pose.header.frame_id = base_frame;
  if (move_cart_action.sendGoalAndWait(ik_goal_, ros::Duration(60.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Failed to move plug onto stowing pose");
    action_server_->setAborted();
    return;
  }

  // open gripper
  ROS_INFO("Waiting for ripper action");
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_msg;
  gripper_msg.command.position = 0.07;
  gripper_msg.command.max_effort = 100000.0;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_action("r_gripper_controller/gripper_action", true);
  gripper_action.waitForServer();
  ROS_INFO("Opening gripper");
  if (gripper_action.sendGoalAndWait(gripper_msg, ros::Duration(10.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Gripper failed to open");
    action_server_->setAborted();
    return;
  }

  // move plug above stowing pose
  ros::Duration(0.5).sleep();
  ROS_INFO("Move plug above stowing pose");
  tf::poseTFToMsg(pose_base_plug*pose_plug_plug*pose_plug_gripper, ik_goal_.pose.pose);
  ik_goal_.pose.header.stamp = ros::Time::now();
  ik_goal_.pose.header.frame_id = base_frame;
  if (move_cart_action.sendGoalAndWait(ik_goal_, ros::Duration(60.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Failed to reach pose above stowing pose");
    action_server_->setAborted();
    return;
  }

  action_server_->setSucceeded();
}
