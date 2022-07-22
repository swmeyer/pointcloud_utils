/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 9 Sept 2021
 * Brief: subscribes to a point cloud 2 message and converts the data into human-readable point and field values
 * File: cloud_reader.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud_utils/pointcloud_utils.hpp"
// --------------------------

class CloudReaderNode : public rclcpp::Node
{
	public:
		CloudReaderNode() :
			Node("cloud_reader_node")
		{
			//main content

			// RCLCPP_INFO(this->get_logger(), "Starting node");

			//Declare parameters
			this->declare_parameter<std::string>("cloud_topic", "/cloud");

			//Get parameters

			std::string cloud_topic;
			this->get_parameter<std::string>("cloud_topic", cloud_topic);

			cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(), std::bind(&CloudReaderNode::pointCloudCallback, this, std::placeholders::_1));
			// RCLCPP_INFO(this->get_logger(), "Cloud topic: %s", cloud_topic.c_str()); 
			
			// clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
			// timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CloudReaderNode::timerCallback, this));
		}

		~CloudReaderNode()
		{

		}

	private:

		//Variables
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
		// rclcpp::Clock::SharedPtr clock;
		// rclcpp::TimerBase::SharedPtr timer;

		//Methods

		// void timerCallback()
		// {
		// 	RCLCPP_INFO(this->get_logger(), "Timer callback");
		// }


		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			// RCLCPP_INFO(this->get_logger(), "New cloud received");
			std::vector<pointcloud_utils::luminarPointstruct> cloud;
			cloud.resize(msg->width);
			memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);


			// RCLCPP_INFO(this->get_logger(), "time,x,y,z,intensity,ring,");
			std::cout <<  ",time,x,y,z,intensity,ring,\n";

			for (uint i = 0; i < cloud.size(); i++)
			{
				std::cout << "," << cloud[i].time << "," 
						  << cloud[i].x << ","
						  << cloud[i].y << ","
						  << cloud[i].z << ","
						  << cloud[i].intensity << ","
						  << cloud[i].ring << ","
						  << "\n";
			}

			std::cout << ",0,0,0,0,0,0,\n,0,0,0,0,0,0,\n";
		}

};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudReaderNode>());
    rclcpp::shutdown();
	return(0);
}