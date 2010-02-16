#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

// Ros Includes
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Actions
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_plugs_msgs/DetectPlugOnBaseAction.h>

// Message Types
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>


/* DetectPlugOnBase Action
		This action is ued when plugging in to detect the plug when it is on the
		magnetic mount behind the base Hokuyo. 

		This used to be done with the tilt laser, but can no longer be done on the
		PR2 Beta. So we now need to do this by positioning the right forearm camera
		so that it can see the plug, then transforming that pose and returning it as
		the action result.

		The pose that the arm will go to and the number of samples before returning
		success will be stored on the parameter server.

		The plug_tracker (located in visual_feature_trackers/outlet_detection) will
		broadcast on /plug_detector/pose a pose with covariance.
*/

class DetectPlugOnBaseAction
{
public:
    
  DetectPlugOnBaseAction(std::string name) : 
    nh_("~"),
    as_(name),
    action_name_(name),
    positioned_to_detect_plug_(false),
    plug_filter_topic_("plug_filter_topic"),
    plug_tracker_topic_("plug_tracker_topic"),
    joint_states_topic_("joint_states_topic"),
    joint_names_(0),
    goal_joint_values_(0)
  {
    // Register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&DetectPlugOnBaseAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&DetectPlugOnBaseAction::preemptCB, this));

    // Get configuration parameters
    nh_.getParam("max_speed", max_speed_);
    nh_.getParam("n_samples", n_samples_);

		// Get arm pose for looking at the plug
    nh_.getParam("arm", arm_);
    nh_.getParam("arm_controller", arm_controller_);

    ROS_DEBUG("Using arm \"%s\" to find the plug on the base.",arm_.c_str());

		// Populate joint names
		// TODO: Replace this with reading an XML-RPC array
		std::string joint_names_str;
    nh_.getParam("target_pose/joint_names", joint_names_str);

		std::istringstream iss(joint_names_str);
		std::copy(std::istream_iterator<std::string>(iss),
				std::istream_iterator<std::string>(),
				std::back_inserter< std::vector<std::string> >(joint_names_));

		// Get joint values from param server
		goal_joint_values_.resize(joint_names_.size());
		current_joint_values_.resize(joint_names_.size());
		for(unsigned int i=0; i<joint_names_.size(); i++) {
			nh_.getParam("target_pose/"+joint_names_[i], goal_joint_values_[i]);
			joint_names_[i] = arm_+"_"+joint_names_[i]+"_joint";
			ROS_DEBUG("Configuring joint: \"%s\"",joint_names_[i].c_str());
		}


		// Initialize client for joint traj action
		joint_traj_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(arm_controller_+"/joint_trajectory_action",true);

		ros::NodeHandle toplevel_nh;		
		// Subscribe to joint state to get the initial arm pose
		joint_state_sub_ = toplevel_nh.subscribe(joint_states_topic_,1,&DetectPlugOnBaseAction::joint_stateCB, this);
		// Subscribe to the plug tracker
		plug_tracker_sub_ = toplevel_nh.subscribe(plug_tracker_topic_, 1,&DetectPlugOnBaseAction::trackerCB, this);
		plug_tracker_pub_ = toplevel_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(plug_filter_topic_, 1);

		ROS_INFO("%s ready",action_name_.c_str());
  }

  ~DetectPlugOnBaseAction(void)
  {
		delete joint_traj_client_;
  }

  void goalCB()
  {
    ROS_INFO("%s: Received Goal", action_name_.c_str() );

		// Compute the duration of the trajectory
		// Get the current configuration of the relevant joints
		sensor_msgs::JointState joint_state;

		double dist = 0;
		for(unsigned int i=0; i < joint_names_.size(); i++) {
			dist = pow(goal_joint_values_[i] - current_joint_values_[i],2)+dist; 
		}
		dist = sqrt(dist);
		ros::Duration traj_duration(dist / max_speed_);

		std::vector<double> joint_velocities(7, 0.0);

		// Computes the command to send to the trajectory controller
		pr2_controllers_msgs::JointTrajectoryGoal goal;

		goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
		goal.trajectory.points.resize(1);
		goal.trajectory.joint_names = joint_names_;
		goal.trajectory.points[0].positions = goal_joint_values_;
		goal.trajectory.points[0].velocities = joint_velocities;
		goal.trajectory.points[0].time_from_start = ros::Duration(traj_duration);
	 
		// Accept goal
		as_.acceptNewGoal();

		// Send sub-goal
		joint_traj_client_->sendGoal(goal);

		// Wait for sub-goal to terminate
		bool finished_before_timeout = joint_traj_client_->waitForResult(ros::Duration(30.0));

		if (finished_before_timeout) {
			// Get movement goal response state
			actionlib::SimpleClientGoalState state = joint_traj_client_->getState();

			// Check for success
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				// Mark that we are in position to detect the plug now
				feedback_.cur_sample = 1;
				result_.pose.pose.pose.position.x = 0;
				result_.pose.pose.pose.position.y = 0;
				result_.pose.pose.pose.position.z = 0;
				result_.pose.pose.pose.orientation.x = 0;
				result_.pose.pose.pose.orientation.y = 0;
				result_.pose.pose.pose.orientation.z = 0;
				result_.pose.pose.pose.orientation.w = 0;
				positioned_to_detect_plug_ = true;
				ROS_INFO("Ready to detect the plug on the base: %s",state.toString().c_str());
			}

		} else  { 
			ROS_ERROR("JointTrajectoryAction did not finish before the time out.");

			as_.setAborted(result_);
		}
	}

  void preemptCB()
  {
		// Stop the arm
		// Return the arm to the previous position

		// Disable the plug tracker over ros
		// TODO: For now it is always active
		// Set preempted
		as_.setPreempted();
  }

	void trackerCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& plug_pose)
	{
    if (!as_.isActive() || !positioned_to_detect_plug_)
      return;

		// Combine pose into running average
		result_.pose.header = plug_pose->header;
		result_.pose.pose.pose = plug_pose->pose.pose;

		// Send feedback
		feedback_.cur_sample++;
		as_.publishFeedback(feedback_);

		// If we have all the samples we care about, terminate the action
		if(feedback_.cur_sample >= n_samples_) {
 			ROS_INFO("Found te plug on the base.");
			plug_tracker_pub_.publish(result_.pose);
			positioned_to_detect_plug_ = false;
			as_.setSucceeded(result_);
		}
	}

	void joint_stateCB(const sensor_msgs::JointState::ConstPtr& joint_state)
	{
		// Disregard if we are active
    if (as_.isActive())
      return;

		//ROS_INFO("Receiving joint state");

		// Store the current relevant joint values
		for(unsigned int i=0; i < joint_names_.size(); i++) {
			std::string joint_name = joint_names_[i];

			for(unsigned int j=0; j<joint_state->name.size(); j++) {
				if(joint_name == joint_state->name[j]) {
					current_joint_values_[i] = joint_state->position[j];
				}
			}
		}
	}


protected:
    
  ros::NodeHandle nh_;

	// Action Server
  actionlib::SimpleActionServer< pr2_plugs_msgs::DetectPlugOnBaseAction > as_;
  std::string action_name_;
  pr2_plugs_msgs::DetectPlugOnBaseFeedback feedback_;
  pr2_plugs_msgs::DetectPlugOnBaseResult result_;

	// Action Clients
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> *joint_traj_client_; 

	// Time to take to untuck and position the arm
	double max_speed_;
	// Flag true when we are positioned
	bool positioned_to_detect_plug_;
	// Number of samples to take
	int n_samples_;

  std::string
		arm_,
		arm_controller_,
		base_frame_,
                plug_filter_topic_,
                plug_tracker_topic_,
                joint_states_topic_;
	std::vector<std::string> joint_names_;
	std::vector<double>
		current_joint_values_,
		goal_joint_values_;

	// Topic subscribers and publishers
  ros::Subscriber
		joint_state_sub_,
		plug_tracker_sub_;

	ros::Publisher
		plug_tracker_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_plug_on_base");

	// Construct the server
  DetectPlugOnBaseAction detect_plug_on_base(ros::this_node::getName());
  ros::spin();

  return 0;
}
