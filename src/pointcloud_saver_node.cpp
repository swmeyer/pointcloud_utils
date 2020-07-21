/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
/* Brief: ROS node to wrap a point cloud saver class
/* File: pointcloud_saver_node.cpp
*/

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_utils/pointcloud_saver.hpp"
// --------------------------

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pointcloud_saver");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	//Get params
	std::string topic;
	n_.param<std::string>("cloud_topic", topic, "/cloud");

	std::string filename;
	n_.param<std::string>("filename_base", filename, "points");

	std::string filetype;
	n_.param<std::string>("file_extension", filetype, ".csv");


	//Make pointcloud svaver object
	PointCloudSaver pc_saver(topic, filename, filetype, n);

	ros::spin();
	return(0);
}