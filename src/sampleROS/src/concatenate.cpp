#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

ros::Publisher pub;
sensor_msgs::PointCloud2 output;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	const sensor_msgs::PointCloud2 curr = output;
  pcl::concatenatePointCloud(curr, *input, output);
  
  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "concatenate");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("buffer", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("out_cloud", 1);
 
  // Spin
  ros::spin ();
}