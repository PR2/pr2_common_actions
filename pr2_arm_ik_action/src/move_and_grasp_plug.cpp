/*********************************************************************       
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
 *   * Neither the name of Willow Garage nor the names of its                
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
 * Author: Melonee Wise
 *********************************************************************/


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_plugs_msgs/MoveAndGraspPlugAction.h>
#include <pr2_plugs_msgs/PR2ArmIKAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

class MoveAndGraspPlugAction
{
public:
    
  MoveAndGraspPlugAction(std::string name) : 
    nh_("~"),
    as_(name),
    action_name_(name),
    tool_frame_("_gripper_tool_frame"),
    base_frame_("base_link")
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&MoveAndGraspPlugAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveAndGraspPlugAction::preemptCB, this));

    // get relevant params
    nh_.param<std::string>("arm", arm_, "r");
    nh_.getParam("ik_action_name", ik_action_name_ );
    nh_.getParam("plug_frame", plug_frame_);
    nh_.getParam("use_preferred_grip_orientation", use_preferred_grip_orientation_);
    nh_.getParam("grip_angle_offset", grip_angle_offset_);
    nh_.getParam("plug_frame_x_offset", plug_x_offset_);
    nh_.getParam("plug_frame_y_offset", plug_y_offset_);
    nh_.getParam("plug_frame_z_offset", plug_z_offset_);
    nh_.getParam("pose_suggestion/shoulder_pan", shoulder_pan_ );
    nh_.getParam("pose_suggestion/shoulder_lift", shoulder_lift_ );
    nh_.getParam("pose_suggestion/upper_arm_roll", upper_arm_roll_);
    nh_.getParam("pose_suggestion/elbow_flex", elbow_flex_);
    nh_.getParam("pose_suggestion/forearm_roll", forearm_roll_);
    nh_.getParam("pose_suggestion/wrist_flex",  wrist_flex_);
    nh_.getParam("pose_suggestion/wrist_roll",  wrist_roll_);
   
    ik_goal_.ik_seed.name.push_back(arm_+"_shoulder_pan_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_shoulder_lift_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_upper_arm_roll_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_elbow_flex_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_forearm_roll_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_wrist_flex_joint");
    ik_goal_.ik_seed.name.push_back(arm_+"_wrist_roll_joint");
    ik_goal_.ik_seed.position.push_back(shoulder_pan_);
    ik_goal_.ik_seed.position.push_back(shoulder_lift_);
    ik_goal_.ik_seed.position.push_back(upper_arm_roll_);
    ik_goal_.ik_seed.position.push_back(elbow_flex_);
    ik_goal_.ik_seed.position.push_back(forearm_roll_);
    ik_goal_.ik_seed.position.push_back(wrist_flex_);
    ik_goal_.ik_seed.position.push_back(wrist_roll_);


    gripper_action_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>(arm_+"_gripper_controller/gripper_action", true);
    while(!gripper_action_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for gripper_action action server to come up");
    }
    ik_action_ = new actionlib::SimpleActionClient<pr2_plugs_msgs::PR2ArmIKAction>(ik_action_name_, true);
    while(!ik_action_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for ik_action action server to come up");
    }

    ROS_INFO("%s: Initialized", action_name_.c_str() );
  }

  ~MoveAndGraspPlugAction(void)
  {
    delete gripper_action_;
    delete ik_action_;
  }

  void goalCB()
  {
    ros::Rate r(100);
    ros::Duration timeout(5.0);
    // accept the new goal 
    pr2_plugs_msgs::MoveAndGraspPlugGoal goal = *as_.acceptNewGoal();
    ROS_INFO("%s: Accepted Goal", action_name_.c_str() );

    // open the girpper
    pr2_controllers_msgs::Pr2GripperCommandGoal gripper_goal;
    gripper_goal.command.position = 0.075;
    gripper_goal.command.max_effort = 2000.0;
    if(gripper_action_->sendGoalAndWait(gripper_goal, ros::Duration(5.0), timeout) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("%s: Aborted: gripper failed to open", action_name_.c_str()); 
      //set the action state to aborted
      as_.setAborted(result_);
      return;
    }      


    //Try to transform the pose to the root link frame
    bool ret1 = false;
    tf::StampedTransform transform_offset, temp_pose_transform;
    tf::Transform target_pose_transform;
    std::string error_msg;
    try 
    {
      ret1 = tf_.waitForTransform(base_frame_, plug_frame_, goal.pose.header.stamp,
                                  ros::Duration(5.0), ros::Duration(0.01), &error_msg);
      // Transforms the pose into the root link frame                                                         
      tf_.lookupTransform(base_frame_, plug_frame_, goal.pose.header.stamp, temp_pose_transform );
      // Add offset pose from plug frame to gripper frame
      target_pose_transform = temp_pose_transform * tf::Transform(tf::createQuaternionFromRPY(M_PI_2, 0.0, grip_angle_offset_),
								  tf::Vector3(plug_x_offset_, plug_y_offset_, plug_z_offset_));
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      as_.setAborted(result_);
      return;
    }
    try 
    {
      ret1 = tf_.waitForTransform(arm_+tool_frame_, arm_+"_wrist_roll_link", goal.pose.header.stamp,
                                  ros::Duration(5.0), ros::Duration(0.01), &error_msg);
      // Transforms the pose into the root link frame                                
      tf_.lookupTransform(arm_+tool_frame_, arm_+"_wrist_roll_link", goal.pose.header.stamp, transform_offset);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      as_.setAborted(result_);
      return;
    }
    tf::poseTFToMsg(target_pose_transform*transform_offset, ik_goal_.pose.pose);
    ik_goal_.pose.header.stamp = goal.pose.header.stamp;
    ik_goal_.pose.header.frame_id = base_frame_;
    
    // move the arm above the plug
    ik_goal_.pose.pose.position.z = ik_goal_.pose.pose.position.z + 0.06; 
    if(ik_action_->sendGoalAndWait(ik_goal_, ros::Duration(50.0), timeout) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("%s: Aborted: arm failed to move", action_name_.c_str());
      //set the action state to aborted      
      as_.setAborted(result_);
      return;
    }
    // move the arm down on to the plug
    ik_goal_.pose.pose.position.z = ik_goal_.pose.pose.position.z - 0.06;
    ik_goal_.pose.header.stamp = ros::Time::now();
    if(ik_action_->sendGoalAndWait(ik_goal_, ros::Duration(50.0), timeout) != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("%s: Aborted: arm failed to move", action_name_.c_str());
      //set the action state to aborted        
      as_.setAborted(result_);
      return;
    }
    //grab the plug
    gripper_goal.command.position = 0.0;
    gripper_goal.command.max_effort = -1.0;
    if(gripper_action_->sendGoalAndWait(gripper_goal, ros::Duration(5.0), timeout) == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("%s: Aborted: gripper failed to grab the plug", action_name_.c_str());
      //set the action state to aborted   
      as_.setAborted(result_);
      return;
    }
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);

  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempt", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }


protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pr2_plugs_msgs::MoveAndGraspPlugAction> as_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* gripper_action_;
  actionlib::SimpleActionClient<pr2_plugs_msgs::PR2ArmIKAction>* ik_action_;
  std::string action_name_, arm_, ik_action_name_, tool_frame_, base_frame_, plug_frame_;
	bool use_preferred_grip_orientation_;
	double grip_angle_offset_;
  double shoulder_pan_, shoulder_lift_, upper_arm_roll_, elbow_flex_, forearm_roll_, wrist_flex_, wrist_roll_, plug_x_offset_, plug_y_offset_, plug_z_offset_; 
  pr2_plugs_msgs::MoveAndGraspPlugResult result_;
  tf::TransformListener tf_;
  pr2_plugs_msgs::PR2ArmIKGoal ik_goal_;
  bool done_;
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_and_grasp_plug");

  MoveAndGraspPlugAction move_and_grasp(ros::this_node::getName());
  ros::spin();

  return 0;
}

