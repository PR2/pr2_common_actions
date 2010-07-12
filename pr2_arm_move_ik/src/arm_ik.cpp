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
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <pr2_common_action_msgs/PR2ArmIKAction.h>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>

class PR2ArmIKAction
{

public:

  PR2ArmIKAction(std::string name) :
    nh_("~"),
    dimension_(7),
    action_name_(name),
    as_(name, boost::bind(&PR2ArmIKAction::executeCB, this, _1))
  {
    //register the goal and feeback callbacks
    as_.registerPreemptCallback(boost::bind(&PR2ArmIKAction::preemptCB, this));

    ros::NodeHandle nh_toplevel;

    // Load Robot Model
    ROS_INFO("%s: Loading robot model", action_name_.c_str());
    std::string xml_string;
    if (!nh_toplevel.getParam(std::string("robot_description"), xml_string))
    {
      ROS_ERROR("Could not find parameter robot_description on parameter server.");
      ros::shutdown();
      exit(1);
    }
    if(!robot_model_.initString(xml_string))
    {
      ROS_ERROR("Could not load robot model.");
      ros::shutdown();
      exit(1);
    }

    // Get Parameters 
    nh_.param<std::string>("arm", arm_, "r");
    nh_.param("joint_trajectory_action", joint_action_, std::string("joint_trajectory_action"));
    nh_.param("free_angle", free_angle_, 2);
    nh_.param("search_discretization", search_discretization_, 0.01);
    nh_.param("ik_timeout", timeout_, 5.0);
    root_name_ = "torso_lift_link";
    tip_name_ = arm_ + "_wrist_roll_link";
    

    // Init pose suggestion
    jnt_pos_suggestion_.resize(dimension_);

    ROS_DEBUG("%s: Loading KDL chain", action_name_.c_str());
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model_, tree))
    {
      ROS_ERROR("%s: Could not load the KDL tree from the robot model", action_name_.c_str());
      ros::shutdown();
      exit(1);
    }
    if (!tree.getChain(root_name_, tip_name_, kdl_chain_))
    {
      ROS_ERROR("%s: Could not create the KDL chain", action_name_.c_str());
      ros::shutdown();
      exit(1);
    }

    // Init IK
    ROS_DEBUG("Starting with search_discretization %f and ik_timeout %f", search_discretization_,timeout_);
    pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model_, root_name_, tip_name_, search_discretization_, free_angle_));

    if(!pr2_arm_ik_solver_->active_)
    {
      ROS_ERROR("%s: Could not load pr2 arm IK solver", action_name_.c_str());
      ros::shutdown();
      exit(1);
    }

    std::string trajectory_action_name = joint_action_;
    trajectory_action_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(trajectory_action_name, true);

    double counter = 0;
    while(!trajectory_action_->waitForServer(ros::Duration(80.0)))
    {
      ROS_DEBUG("%s: Waiting for %s action server to come up", action_name_.c_str(), trajectory_action_name.c_str());
      counter++;
      if(counter > 3)
      {
        ROS_ERROR("%s: %s action server took too long to start", action_name_.c_str(), trajectory_action_name.c_str());
        //set the action state to aborted
        ros::shutdown();
        exit(1);
      }
    }
    //Action ready
    ROS_INFO("%s: Initialized", action_name_.c_str());
  }

  ~PR2ArmIKAction(void)
  {
    delete trajectory_action_;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempt", action_name_.c_str());
    // set the action state to preempted
    trajectory_action_->cancelGoal();
    as_.setPreempted();
  }

  void executeCB(const pr2_common_action_msgs::PR2ArmIKGoalConstPtr & goal)
  {
    // accept the new goal
    ROS_INFO("%s: Accepted Goal", action_name_.c_str() );
   
    //Try to transform the pose to the root link frame
    bool ret1 = false;
    tf::Stamped<tf::Pose> tf_pose_stamped;
    KDL::Frame desired_pose;
    try
    {
      std::string error_msg;
      ret1 = tf_.waitForTransform(root_name_, goal->pose.header.frame_id, goal->pose.header.stamp,
				  ros::Duration(5.0), ros::Duration(0.01), &error_msg);
      // Transforms the pose into the root link frame
      tf::poseStampedMsgToTF(goal->pose, tf_pose_stamped);
      tf_.transformPose(root_name_, tf_pose_stamped, tf_pose_stamped);
      tf::PoseTFToKDL(tf_pose_stamped, desired_pose);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      as_.setAborted();
      return;
    }

    // Get the IK seed from the goal 
    for(int i=0; i < dimension_; i++)
    {
      jnt_pos_suggestion_(getJointIndex(goal->ik_seed.name[i])) = goal->ik_seed.position[i];
    }

    ROS_DEBUG("calling IK solver");
    KDL::JntArray jnt_pos_out;
    bool is_valid = (pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_suggestion_, desired_pose, jnt_pos_out, timeout_)>=0);
    if(!is_valid)
    {
      ROS_ERROR("%s: Aborted: IK invalid", action_name_.c_str());
      //set the action state to aborted
      as_.setAborted();
      return;
    }

    std::vector<double> traj_desired(dimension_);
    std::vector<std::string> traj_names(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      traj_names[i] = goal->ik_seed.name[i];
      traj_desired[i] = jnt_pos_out(getJointIndex(goal->ik_seed.name[i]));
    }

    pr2_controllers_msgs::JointTrajectoryGoal traj_goal ;
    std::vector<double> velocities(dimension_, 0.0);
    traj_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.03);
    traj_goal.trajectory.points.resize(1);
    traj_goal.trajectory.joint_names = traj_names;

    traj_goal.trajectory.points[0].positions = traj_desired;
    traj_goal.trajectory.points[0].velocities = velocities;
    traj_goal.trajectory.points[0].time_from_start = goal->move_duration;

    // Send goal
    trajectory_action_->sendGoal(traj_goal);
    trajectory_action_->waitForResult();
    if(trajectory_action_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("%s: Aborted: trajectory action failed to achieve goal", action_name_.c_str());
      //set the action state to aborted
      as_.setAborted(result_);
      return;
    }

    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }


  int getJointIndex(const std::string &name)
  {
    int i=0; // segment number
    int j=0; // joint number
    while(j < dimension_ && i < (int) kdl_chain_.getNrOfSegments())
    {
      if(kdl_chain_.getSegment(i).getJoint().getType() == KDL::Joint::None)
      {
        i++;
        continue;
      }
      if(kdl_chain_.getSegment(i).getJoint().getName() == name)
      {
        return j;
      }
      i++;
      j++;
    }
    return -1;
  }


protected:

  ros::NodeHandle nh_;
  urdf::Model robot_model_;
  std::string joint_action_;
  int dimension_, free_angle_;
  double search_discretization_, timeout_;
  std::string action_name_, arm_, arm_controller_, root_name_, tip_name_;

  KDL::Chain kdl_chain_;
  KDL::JntArray jnt_pos_suggestion_;

  actionlib::SimpleActionServer<pr2_common_action_msgs::PR2ArmIKAction> as_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* trajectory_action_;

  boost::shared_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_arm_ik_solver_;
  tf::TransformListener tf_;

  pr2_common_action_msgs::PR2ArmIKResult result_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arm_ik");

  PR2ArmIKAction pr2_arm_ik(ros::this_node::getName());
  ros::spin();

  return 0;
}
