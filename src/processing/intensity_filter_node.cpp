/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 8 Sept 2020
 * Brief: Implementation of a class to republishes the incoming point cloud as a filtered cloud containing only the points with intensity
 *        within a given threshold range
 * File: intensity_filter_node.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/processing/intensity_filter.hpp>
// --------------------------

// --------------------------
int intensity_min;
int intensity_max;

ros::Publisher lidar_pub;

pointcloud_utils::IntensityFilter* intensity_filter;
// --------------------------

/**
 * @Function 	pointCloudCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::PointCloud2 filtered_cloud;
	std::vector<pointcloud_utils::pointstruct> point_vector;

	intensity_filter->filterIntensity(msg, point_vector, filtered_cloud, intensity_min, intensity_max);

	lidar_pub.publish(filtered_cloud);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "distance_publisher");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string lidar_sub_topic, lidar_pub_topic;
	n_.param<std::string>("cloud_topic_in", lidar_sub_topic, "/velodyne_points");
	n_.param<std::string>("cloud_topic_out", lidar_pub_topic, "/filtered_points");

	n_.param<int>("intensity_min", intensity_min, 20);
	n_.param<int>("intensity_max", intensity_max, 250);

	intensity_filter = new pointcloud_utils::IntensityFilter();

	lidar_pub = n.advertise<sensor_msgs::PointCloud2>(lidar_pub_topic, 1);
	
	ros::Subscriber lidar_sub = n.subscribe(lidar_sub_topic, 1, &pointCloudCallback);

	ros::spin();

	delete intensity_filter;

	return 0;
}