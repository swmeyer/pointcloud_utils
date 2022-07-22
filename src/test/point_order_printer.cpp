/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 9 Sept 2021
 * Brief: subscribes to a point cloud 2 message and prints out markers representing the points in the order they are stored in the cloud
 * File: point_order_printer.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
	
#include <string>
#include <math.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pointcloud_utils/pointcloud_utils.hpp"
// --------------------------

class PointOrderPrinterNode : public rclcpp::Node
{
	public:
		PointOrderPrinterNode() :
			Node("point_order_printer"),
			cloud_num(0),
			processing(false)
		{
			//main content

			//Declare parameters
			this->declare_parameter<std::string>("cloud_topic", "/cloud");
			this->declare_parameter<std::string>("visualization_topic", "/point_markers");
			this->declare_parameter<float>("point_size", 2.0);
			this->declare_parameter<float>("fill_time", 1.0);
			this->declare_parameter<bool>("print_one_ring", false);
			this->declare_parameter<int>("ring_id", 0);
			this->declare_parameter<bool>("const_depth", false);
			this->declare_parameter<double>("depth_value", 1.0);

			//Get parameters

			std::string cloud_topic, visualization_topic;
			this->get_parameter<std::string>("cloud_topic", cloud_topic);
			this->get_parameter<std::string>("visualization_topic", visualization_topic);
			this->get_parameter<float>("point_size", this->point_size);
			this->get_parameter<float>("fill_time", this->fill_time);
			this->get_parameter<bool>("print_one_ring", this->print_one_ring);
			this->get_parameter<int>("ring_id", this->ring_id);
			this->get_parameter<bool>("const_depth", this->const_depth);
			this->get_parameter<double>("depth_value", this->depth_value);


			cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(), std::bind(&PointOrderPrinterNode::pointCloudCallback, this, std::placeholders::_1));
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 100);
		}

		~PointOrderPrinterNode()
		{

		}

	private:

		//Variables
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

		float fill_time; //[s] amount of time for each point to wait before publishing
		float point_size; //size of published marker points

		std_msgs::msg::Header header; //Saved header from last point cloud

		int cloud_num;
		bool processing;
		
		bool print_one_ring; //if true, only print the specified ring
		int ring_id; //id of the ring to print
		bool const_depth; //if true, print only the polar angle positions using a constant range
		double depth_value; //[m] constant depth to print at


		//Methods

		/**
		 * @Function 	publishPoints
		 * @Param 		cloud - point cloud to publish points from
		 * @Return 		void
		 * @Brief 		Publishes the given cloud in the order the points are stored in
		 */
		void publishPoints(const std::vector<pointcloud_utils::luminarPointstruct>& cloud)
		{
			this->processing = true;

			visualization_msgs::msg::MarkerArray marker_array_msg;

			visualization_msgs::msg::Marker marker;
			marker.header = this->header;
			// marker.id = 0;
			// marker.ns = "";
			marker.ns = std::to_string(this->cloud_num);
			// marker.lifetime = rclcpp::Duration(this->decay_time);
			marker.type = visualization_msgs::msg::Marker::POINTS;
			marker.action = visualization_msgs::msg::Marker::ADD;

			marker.scale.x = this->point_size;
			marker.scale.y = this->point_size;

			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;


			geometry_msgs::msg::Point point;

			for (uint i = 0; i < cloud.size(); i++)
			{				
				if (!rclcpp::ok()) break;

				if (this->print_one_ring)
				{
					if (cloud[i].ring != this->ring_id) continue;
				}

				marker.id = i;
				// marker.ns = std::to_string(cloud[i].ring);

				// RCLCPP_INFO(this->get_logger(), "Point: %d", i );

				point.x = cloud[i].x;
				point.y = cloud[i].y;
				point.z = cloud[i].z;

				if (this->const_depth)
				{
					double norm = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2)) / this->depth_value;
					
					point.x = point.x / norm;
					point.y = point.y / norm;
					point.z = point.z / norm;
				}

				marker.points.clear();
				marker.points.push_back(point);

				marker_array_msg.markers.push_back(marker);

				this->marker_pub->publish(marker_array_msg);
				marker_array_msg.markers.clear();
				int elapse = this->fill_time * 1000000000; //s to ns
				rclcpp::sleep_for(std::chrono::nanoseconds(elapse));

			}

			this->cloud_num++;

			this->processing = false;

			// RCLCPP_INFO(this->get_logger(), "Finished");

		}

		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			if (this->processing) return;

			// RCLCPP_INFO(this->get_logger(), "New Cloud");

			std::vector<pointcloud_utils::luminarPointstruct> cloud;
			cloud.resize(msg->width);
			memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);

			this->header = msg->header;

			publishPoints(cloud);
		}


};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointOrderPrinterNode>());
    rclcpp::shutdown();
	return(0);
}