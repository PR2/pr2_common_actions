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
#include <pr2_arm_ik/pr2_arm_ik_solver.h>
#include <pr2_arm_ik_action/trajectory_unwrap.h>
#include <pr2_arm_ik_action/trajectory_generation.h>
#include <pr2_common_action_msgs/PR2ArmIKAction.h>
#include <kdl/frames_io.hpp>

class PR2ArmIKAction
{

public:

  PR2ArmIKAction(std::string name) :
    nh_("~"),
    dimension_(7),
    action_name_(name),
    as_(name),
    generator_(2.5, 2.0, dimension_)
  {
    //register the goal and feeback callbacks                                                            
    as_.registerGoalCallback(boost::bind(&PR2ArmIKAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PR2ArmIKAction::preemptCB, this));

    // Load Robot Model                                                                                                            
    std::string urdf_xml, full_urdf_xml, xml_string;
    nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
    nh_.searchParam(urdf_xml, full_urdf_xml);
    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (nh_.getParam(full_urdf_xml, result))
      xml.Parse(result.c_str());
    else
      {
	ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
	ros::shutdown();
	exit(1);
      }
    xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if (!root || !root_element)
      {
	ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
	ros::shutdown();
	exit(1);
      }
    robot_model.initXml(root);

    // Which arm?                                                                                                                  
    nh_.param<std::string>("arm", arm_, "r");
    nh_.getParam("arm_controller", arm_controller_ );
    root_name_ = "torso_lift_link";
    tip_name_ = arm_ + "_wrist_roll_link";


    ROS_DEBUG("Loading KDL chain");
    if(!pr2_arm_ik::getKDLChain(xml_string, root_name_, tip_name_, kdl_chain_))
      {
	ROS_ERROR("Could not load kdl chain");
	ros::shutdown();
	exit(1);
      }
    // Init IK                                                                                                                                              
    nh_.param("free_angle", free_angle_, 2);
    nh_.param("search_discretization", search_discretization_, 0.01);
    nh_.param("ik_timeout", timeout_, 5.0);
    nh_.param("max_velocity", max_velocity_, 0.5);
    ROS_INFO("Starting arm ik action with search_discretization %f, ik_timeout %f and max_velocity %f", search_discretization_,timeout_,max_velocity_);
    pr2_arm_ik_solver_.reset(new pr2_arm_ik::PR2ArmIKSolver(robot_model, root_name_, tip_name_, search_discretization_, free_angle_));

    if(!pr2_arm_ik_solver_->active_)
      {
	ROS_ERROR("Could not load ik");
	ros::shutdown();
	exit(1);
      }

    // Init pose suggestion              
    jnt_pos_suggestion_.resize(dimension_);

    ros::NodeHandle nh_toplevel;
    query_traj_srv_ = nh_toplevel.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>(arm_controller_+"/query_state");
    trajectory_action_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(arm_controller_+"/joint_trajectory_action", true);
    double counter = 0;
    while(!trajectory_action_->waitForServer(ros::Duration(80.0)))
    {
      ROS_INFO("%s: Waiting for trajectory_action action server to come up", action_name_.c_str());
      counter++;
      if(counter > 3)
      {
        ROS_ERROR("%s: Aborted: joint_trajectory_action action server took too long to come up", action_name_.c_str());
        //set the action state to aborted                
        as_.setAborted(result_);
        return;
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
    as_.setPreempted();
  }
  
  void goalCB()
  {
    // accept the new goal                                                      
    pr2_common_action_msgs::PR2ArmIKGoal goal = *as_.acceptNewGoal();
    ROS_INFO("%s: Accepted Goal", action_name_.c_str() );
     
    //Try to transform the pose to the root link frame                                                                                                      
    bool ret1 = false;
    tf::Stamped<tf::Pose> tf_pose_stamped;
    ROS_DEBUG("computing transform from the goal to the base link");
    try 
    {
      std::string error_msg;
      ret1 = tf_.waitForTransform(root_name_, goal.pose.header.frame_id, goal.pose.header.stamp,
				  ros::Duration(5.0), ros::Duration(0.01), &error_msg);
      // Transforms the pose into the root link frame   
      tf::poseStampedMsgToTF(goal.pose, tf_pose_stamped);
      tf_.transformPose(root_name_, tf_pose_stamped, tf_pose_stamped);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
      as_.setAborted();
      return;
    }
    KDL::Frame desired_pose;
    tf::PoseTFToKDL(tf_pose_stamped, desired_pose);


    //get the IK seed 
    for(int i=0; i < dimension_; i++)
    {
      jnt_pos_suggestion_(getJointIndex(goal.ik_seed.name[i])) = goal.ik_seed.position[i];
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

    //Now put the jnt commands back in the correct order for the trajectory controller      
    pr2_controllers_msgs::QueryTrajectoryState traj_state;
    traj_state.request.time = ros::Time::now();
    if (!query_traj_srv_.call(traj_state))
    {
      ROS_ERROR("%s: Aborted: service call to query controller trajectory failed", action_name_.c_str());
      as_.setAborted(result_);
      return;
    }
    std::vector<double> traj_desired(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      traj_desired[i] = jnt_pos_out(getJointIndex(traj_state.response.name[i]));
    }

    pr2_controllers_msgs::JointTrajectoryGoal traj_goal ;
    std::vector<double> velocities(dimension_, 0.0);
    traj_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.03);
    traj_goal.trajectory.points.resize(2);
    traj_goal.trajectory.joint_names = traj_state.response.name;

    traj_goal.trajectory.points[0].positions = traj_state.response.position;
    traj_goal.trajectory.points[0].velocities = velocities;

    traj_goal.trajectory.points[1].positions = traj_desired;
    traj_goal.trajectory.points[1].velocities = velocities;
    ROS_DEBUG("filled out trajectory");

    //unwrap angles   
    trajectory_unwrap::unwrap(robot_model, traj_goal.trajectory,traj_goal.trajectory);  

    //Compute the duration of the trajectory                                                    
    if(goal.move_duration == ros::Duration(0.0)) 
    {
	    double dist = 0;
	    for(int i=0; i < dimension_; i++)
		    dist = pow(traj_goal.trajectory.points[1].positions[i] - traj_state.response.position[i],2)+dist;
	    dist = sqrt(dist);
	    traj_goal.trajectory.points[1].time_from_start = ros::Duration(dist / max_velocity_);
    } 
    else 
    {
	    traj_goal.trajectory.points[1].time_from_start = goal.move_duration;
    }

    generator_.generate(traj_goal.trajectory,traj_goal.trajectory);
    ROS_DEBUG("trajectory duration %f", traj_goal.trajectory.points.back().time_from_start.toSec());

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
  urdf::Model robot_model;
  int dimension_, free_angle_;
  double search_discretization_, timeout_, max_velocity_;
  std::string action_name_, arm_, arm_controller_, root_name_, tip_name_;

  KDL::Chain kdl_chain_;
  KDL::JntArray jnt_pos_suggestion_;

  actionlib::SimpleActionServer<pr2_common_action_msgs::PR2ArmIKAction> as_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* trajectory_action_;

  boost::shared_ptr<pr2_arm_ik::PR2ArmIKSolver> pr2_arm_ik_solver_;
  tf::TransformListener tf_;
  ros::ServiceClient query_traj_srv_;

  pr2_common_action_msgs::PR2ArmIKResult result_;
  trajectory::TrajectoryGenerator generator_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arm_ik");

  PR2ArmIKAction pr2_arm_ik(ros::this_node::getName());
  ros::spin();

  return 0;
}
