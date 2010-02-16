#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_plugs_msgs/MoveAndGraspPlugAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_move_and_grasp_plug");

  // create the action client                                                                                               
  // true causes the client to spin it's own thread                                                                         
  actionlib::SimpleActionClient<pr2_plugs_msgs::MoveAndGraspPlugAction> ac("move_and_grasp_plug", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start                                                                                    
  ac.waitForServer(); //will wait for infinite time                                                                         

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action                                                                                              
  pr2_plugs_msgs::MoveAndGraspPlugGoal goal;
  /*goal.pose.header.frame_id = "base_link";
  goal.pose.header.stamp = ros::Time::now();
  goal.pose.pose.pose.orientation.x = -0.064;
  goal.pose.pose.pose.orientation.y = 0.100;
  goal.pose.pose.pose.orientation.z = 0.685;
  goal.pose.pose.pose.orientation.w = 0.719;
  goal.pose.pose.pose.position.x = 0.242;
  goal.pose.pose.pose.position.y = -0.002;
  goal.pose.pose.pose.position.z = 0.325;*/

  goal.pose.header.frame_id = "l_forearm_cam_optical_frame";
  goal.pose.header.stamp = ros::Time::now();
  goal.pose.pose.pose.orientation.x = 0.5;
  goal.pose.pose.pose.orientation.y = 0.5;
  goal.pose.pose.pose.orientation.z = -0.5;
  goal.pose.pose.pose.orientation.w = -0.5;
  goal.pose.pose.pose.position.x = 0.249;
  goal.pose.pose.pose.position.y = 0.004;
  goal.pose.pose.pose.position.z = 0.34;
  

  /*goal.pose.header.frame_id = "plug_ground_truth";
  goal.pose.header.stamp = ros::Time::now();
  goal.pose.pose.pose.orientation.x = 0.0;
  goal.pose.pose.pose.orientation.y = 0.0;
  goal.pose.pose.pose.orientation.z = 0.0;
  goal.pose.pose.pose.orientation.w = 1.0;
  goal.pose.pose.pose.position.x = 0.0;
  goal.pose.pose.pose.position.y = 0.0;
  goal.pose.pose.pose.position.z = 0.0;
  */
  
  
  ac.sendGoal(goal);

  //wait for the action to return                                                                                           
  bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit                                                                                                                    
  return 0;
}
