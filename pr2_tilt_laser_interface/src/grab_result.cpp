#include <ros/ros.h>
#include <pr2_tilt_laser_interface/GetSnapshotActionResult.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

int 
  main (int argc, char** argv)
{
  ros::init (argc, argv, "result_grabber");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("snapshot", 1, true);

  while (nh.ok ())
  {
    ROS_INFO ("Waiting for cloud to come in over ROS");
    pr2_tilt_laser_interface::GetSnapshotActionResultConstPtr action_result = ros::topic::waitForMessage<pr2_tilt_laser_interface::GetSnapshotActionResult>("get_laser_snapshot/result");
    sensor_msgs::PointCloud2 cloud = action_result->result.cloud;

    pub.publish (cloud);
    ROS_INFO ("Published the point cloud!\n");
    ros::spinOnce ();
    ros::Duration (0.5).sleep ();
  }

  return (0);
}
