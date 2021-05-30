/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
/* Brief: ROS node to wrap a point cloud saver class
/* File: pointcloud_saver_node.cpp
*/

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "pointcloud_utils/io/pointcloud_saver.hpp"
// --------------------------

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pointcloud_saver");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	//Get params
	std::string lidar_topic;
	n_.param<std::string>("cloud_topic", lidar_topic, "/cloud");

	std::string filename;
	n_.param<std::string>("filename_base", filename, "points");

	std::string filetype;
	n_.param<std::string>("file_extension", filetype, ".csv");

	bool from_bag;
	n_.param<bool>("parse_from_bag", from_bag, false);
	std::string bagfile_name;
	n_.param<std::string>("bagfile", bagfile_name, "points.bag");



	//Make pointcloud svaver object
	PointCloudSaver pc_saver(lidar_topic, filename, filetype, n);

	if (from_bag)
	{
    	rosbag::Bag bag;
		//std::string bagstring = "/home/stephanie/Documents/data/fp_bags_truckcomputer/day_2/day2_run1_part2_restamp.bag";
    	std::cout << "Opening bag: " << bagfile_name  << "\n";
	
    	bag.open(bagfile_name);  // BagMode is Read by default
    	
    	for(rosbag::MessageInstance const m: rosbag::View(bag, rosbag::TopicQuery(lidar_topic)))
    	{
    	  sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
    	  if (i != nullptr)
    	  {    	    
    	    pc_saver.setCurrentCloud(i);
    	  }
    	  if (!ros::ok())
    	  {
    	      break;
    	  }
    	}

    	bag.close();
    }

	ros::spin();

	pc_saver.saveTimesToFile();

	return(0);
}