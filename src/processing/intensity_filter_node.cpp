/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 8 Sept 2020
 * Brief: Implementation of a class to republish the incoming point cloud as a filtered cloud containing only the points with intensity
 *        within a given threshold range
 * File: intensity_filter_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud_2.hpp>
#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/processing/intensity_filter.hpp>
// --------------------------

class IntensityFilterNode : public rclcpp:Node
{
	public:
		IntensityFilterNode() :
			Node("intensity_filter_node")
		{
			//main content

			//Declare parameters
			this->declare_parameter<std::string>("cloud_topic_in", "/velodyne_points");
			this->declare_parameter<std::string>("cloud_topic_out", "/filtered_points");

			this->declare_parameter<int>("intensity_min", 20);
			this->declare_parameter<int>("intensity_max", 250);


			//Get parameters
			std::string lidar_sub_topic, lidar_pub_topic;
			this->get_parameter<std::string>("cloud_topic_in", lidar_sub_topic);
			this->get_parameter<std::string>("cloud_topic_out", lidar_pub_topic);

			this->get_parameter<int>("intensity_min", intensity_min);
			this->get_parameter<int>("intensity_max", intensity_max);

			intensity_filter = new pointcloud_utils::IntensityFilter();

			lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_pub_topic, 1);
	
			lidar_sub = n.subscribe(lidar_sub_topic, 1, std::bind(&IntensityFilterNode::pointCloudCallback, this, std::placeholders::_1));

		}

		~IntensityFilterNode()
		{
			delete intensity_filter;
		}

	private:
		//Variables

		int intensity_min;
		int intensity_max;
		
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;

		pointcloud_utils::IntensityFilter* intensity_filter;


		//Methods

		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
		{
			sensor_msgs::msg::PointCloud2 filtered_cloud;
			std::vector<pointcloud_utils::pointstruct> point_vector;
		
			intensity_filter->filterIntensity(msg, point_vector, filtered_cloud, intensity_min, intensity_max);
		
			lidar_pub->publish(filtered_cloud);
		}
}