/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
 * Brief: ROS node to wrap a point cloud saver class
 * File: pointcloud_saver_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud_utils/io/pointcloud_saver.hpp"
// --------------------------

class PointCloudSaverNode : public rclcpp::Node
{
	public:
		PointCloudSaverNode() : 
			Node("pointcloud_saver_node")
		{
			//main content

			//Declare params
			this->declare_parameter<std::string>("cloud_topic", "/luminar_points");
			this->declare_parameter<std::string>("filename_base", "points");
			this->declare_parameter<std::string>("file_extension", ".csv");
			this->declare_parameter<bool>("parse_from_bag", false);
			this->declare_parameter<std::string>("bagfile", "points.bag");
		

			//Get params
			std::string lidar_topic;
			this->get_parameter<std::string>("cloud_topic", lidar_topic);
		
			std::string filename;
			this->get_parameter<std::string>("filename_base", filename);
		
			std::string filetype;
			this->get_parameter<std::string>("file_extension", filetype);
		
			bool from_bag;
			this->get_parameter<bool>("parse_from_bag", from_bag);
			std::string bagfile_name;
			this->get_parameter<std::string>("bagfile", bagfile_name);
		
			//Make pointcloud svaver object
			this->pc_saver = new pointcloud_utils::PointCloudSaver(filename, filetype);

			this->cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudSaverNode::pointCloudCallback, this, std::placeholders::_1));

		}

		~PointCloudSaverNode()
		{
			// this->pc_saver->saveTimesToFile();
			delete this->pc_saver;
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;  		//subscriber for the cloud to save
		pointcloud_utils::PointCloudSaver* pc_saver;

		//Methods:
		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			//memcpy into a struct (parses the data into the struct values)
			// std::vector<pointcloud_utils::luminarPointstruct> cloud;
			// cloud.resize(msg->width);
			// std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);
		
			//call save to file
			// savePointsToFile(cloud);
		
			// this->times.push_back(msg->header.stamp);

			this->pc_saver->setCurrentCloud(msg);
		}

};


int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSaverNode>());
    rclcpp::shutdown();
	return(0);
}