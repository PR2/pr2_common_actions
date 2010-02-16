#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_plugs_msgs/DetectPlugOnBaseAction.h>
#include <pr2_plugs_msgs/MoveAndGraspPlugAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_detect_plug_on_base");

  // create the action client                                                                                               
  // true causes the client to spin it's own thread                                                                         
  actionlib::SimpleActionClient<pr2_plugs_msgs::DetectPlugOnBaseAction> detect_ac("detect_plug_on_base_server", true);
  actionlib::SimpleActionClient<pr2_plugs_msgs::MoveAndGraspPlugAction> grasp_ac("move_and_grasp_plug", true);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start                                                                                    
  detect_ac.waitForServer(); //will wait for infinite time               
  grasp_ac.waitForServer(); //will wait for infinite time              

 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action                                                                                              
  pr2_plugs_msgs::DetectPlugOnBaseGoal goal;

  detect_ac.sendGoal(goal);

  //wait for the action to return                                                                                           
  bool finished_before_timeout = detect_ac.waitForResult(ros::Duration(80.0));

  pr2_plugs_msgs::DetectPlugOnBaseResult detect_result = *detect_ac.getResult();
  pr2_plugs_msgs::MoveAndGraspPlugGoal grasp_goal;
  grasp_ac.sendGoal(grasp_goal);
  finished_before_timeout = grasp_ac.waitForResult(ros::Duration(80.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = grasp_ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit                                                                                                                    
  return 0;
}
