/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 9 Sept 2021
 * Brief: subscribes to a point cloud 2 message and prints out markers representing the rings
 * File: ring_splitter.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
	
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pointcloud_utils/pointcloud_utils.hpp"
// --------------------------

class RingSplitterNode : public rclcpp::Node
{
	public:
		RingSplitterNode() :
			Node("ring_splitter_node"),
			first(true)
		{
			//main content
			// std::cout << "Starting node\n";
			std::cout << "x, y, z, intensity, ring\n";

			//Declare parameters
			this->declare_parameter<std::string>("cloud_topic", "/cloud");
			this->declare_parameter<std::string>("visualization_topic", "/ring_markers");
			this->declare_parameter<int>("ring_group_size", 10);
			this->declare_parameter<float>("point_size", 2.0);

			//Get parameters

			std::string cloud_topic, visualization_topic;
			this->get_parameter<std::string>("cloud_topic", cloud_topic);
			this->get_parameter<std::string>("visualization_topic", visualization_topic);
			this->get_parameter<int>("ring_group_size", this->ring_group_size);
			this->get_parameter<float>("point_size", this->point_size);

			cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS(), std::bind(&RingSplitterNode::pointCloudCallback, this, std::placeholders::_1));
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 1);
		}

		~RingSplitterNode()
		{

		}

	private:

		//Variables
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

		int ring_group_size; //Number of rings to include in a visualization point set
		float point_size; //size of published marker points

		std_msgs::msg::Header header; //Saved header from last point cloud

		bool first;

		//Methods

		/**
		 * @Function 	getMarkerIndex
		 * @Param 		ring - ring number to re-index
		 * @Return 		int - index of the ring group for this ring
		 * @Brief 		reindexes the given ring into its n-member ring group
		 */
		int getMarkerIndex(const int& ring)
		{
			return (int) ring / this->ring_group_size;
		}

		/**
		 * @Function 	publishRings
		 * @Param 		cloud - point cloud to publish rings from
		 * @Return 		void
		 * @Brief 		Parses out the requested ring groups and publishes an array of point set markers
		 */
		void publishRings(const std::vector<pointcloud_utils::luminarPointstruct>& cloud)
		{
			visualization_msgs::msg::MarkerArray marker_array_msg;

			visualization_msgs::msg::Marker marker;
			marker.header = this->header;
			// marker.id = 0;
			marker.type = visualization_msgs::msg::Marker::POINTS;
			marker.action = visualization_msgs::msg::Marker::ADD;

			marker.scale.x = this->point_size;
			marker.scale.y = this->point_size;

			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;

			geometry_msgs::msg::Point point;

			// RCLCPP_INFO(this->get_logger(), "Header frame: %s", this->header.frame_id.c_str());

			uint idx;
			int ring;

			for (uint i = 0; i < cloud.size(); i++)
			{
				if (!rclcpp::ok()) break;

				ring = cloud[i].ring;

				idx = getMarkerIndex(ring);

				// RCLCPP_INFO(this->get_logger(), "IDX: %d", idx);

				if (idx >= marker_array_msg.markers.size())
				{
					marker_array_msg.markers.resize(idx +1);
				}

				if (marker_array_msg.markers[idx].points.size() == 0)
				{
					marker.id = idx;
					marker.ns = std::to_string(idx);
					marker_array_msg.markers[idx] = marker;
				}

				point.x = cloud[i].x;
				point.y = cloud[i].y;
				point.z = cloud[i].z;

				marker_array_msg.markers[idx].points.push_back(point);

			}

			if (marker_array_msg.markers.size() != 0)
			{
				this->marker_pub->publish(marker_array_msg);
			}

		}

		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{

			std::vector<pointcloud_utils::luminarPointstruct> cloud;
			cloud.resize(msg->width);
			memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);

			if (first)
			{
				for (pointcloud_utils::luminarPointstruct pt : cloud)
				{
					std::cout << pt.x << "," << pt.y << "," << pt.z << "," << pt.intensity << "," << pt.ring << "\n";
				}
			}

			first = false;

			this->header = msg->header;

			publishRings(cloud);
		}


};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RingSplitterNode>());
    rclcpp::shutdown();
	return(0);
}