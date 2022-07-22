/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 1 Jun 2020
 * Brief: implementation of a converter for a 3D pointcloud into a 2D laserscan message
 * File: pointcloud_to_laserscan_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud_2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pointcloud_utils/conversion/pointcloud_to_laserscan.hpp>
#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

class PointCloudToLaserScanNode : public rclcpp::Node
{
	public:
		PointCloudToLaserScanNode() :
			Node("pointcloud_to_laserscan_node")
		{
			//main content

			//declare paramters
			this->declare_parameter<std::string>("cloud_sub_topic", "/cloud");
			this->declare_parameter<std::string>("scan_pub_topic",  "/scan");
		
			this->declare_parameter<float>("target_vertical_angle", 0.0);
			this->declare_parameter<float>("vertical_angle_tolerance", 0.1);
			this->declare_parameter<bool>("filter_by_height", false);
			this->declare_parameter<float>("target_height", 0.0);
			this->declare_parameter<float>("height_tolerance", 0.1);
			
			this->declare_parameter<bool>("use_fixed_resolution", false);
    		this->declare_parameter<float>("scan_resultion", 0.001);
		
			this->declare_parameter<std::string>("scan_frame", "scan");
			
			this->declare_parameter<bool>("visualize_3D_scan", false); //set to true to turn on visualization for verify poinstruct parsing
		
			this->declare_parameter<float>("min_angle", -pointcloud_utils::PI);
			this->declare_parameter<float>("max_angle", pointcloud_utils::PI);
		
			this->declare_parameter<bool>("limit_front_distance", false);
			this->declare_parameter<float>("front_distance_limit", 100.0);
			

			//set parameters
			std::string cloud_sub_topic;
			std::string scan_pub_topic;
			this->get_parameter<std::string>("cloud_sub_topic", cloud_sub_topic);
			this->get_parameter<std::string>("scan_pub_topic",  scan_pub_topic);
		
			this->get_parameter<float>("target_vertical_angle", settings.target_vertical_angle);
			this->get_parameter<float>("vertical_angle_tolerance", settings.vertical_angle_tolerance);
			this->get_parameter<bool>("filter_by_height", settings.filter_by_height);
			this->get_parameter<float>("target_height", settings.target_height);
			this->get_parameter<float>("height_tolerance", settings.height_tolerance);
			
			this->get_parameter<bool>("use_fixed_resolution", settings.use_fixed_resolution);
    		this->get_parameter<float>("scan_resultion", settings.resolution);
		
			this->get_parameter<std::string>("scan_frame", scan_frame);
			settings.scan_frame = scan_frame;
			
			this->get_parameter<bool>("visualize_3D_scan", visualize_3D, false); //set to true to turn on visualization for verify poinstruct parsing
		
			this->get_parameter<float>("min_angle", settings.min_angle, -pointcloud_utils::PI);
			this->get_parameter<float>("max_angle", settings.max_angle, pointcloud_utils::PI);
		
			this->get_parameter<bool>("limit_front_distance", settings.limit_front_distance, false);
			this->get_parameter<float>("front_distance_limit", settings.front_distance_limit, 100.0);
			
			converter = new pointcloud_utils::PointCloudToLaserScanConverter(settings);
		
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cloud_markers", 1);
			
			scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_pub_topic, 1);
			cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_sub_topic, 1, std::bind(&PointCloudGridParserNode::cloudCallback, this, std::placeholders::_1));
		
		}

		~PointCloudToLaserScanNode()
		{
			delete converter;
		}

	private:
		//Variables
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
		std::string scan_frame;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; // publisher for visualization
		
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

		pointcloud_utils::PointCloudToLaserScanConverter* converter;
		pointcloud_utils::PointCloudToLaserScanConverter::Settings settings;
		
		bool visualize_3D; //set to true to visualize the converted 3D point coud, to verify poinstruct parsing

		//Methods

		//========================================
		/// @fn         publish_points
		/// @brief      publishes visualization for the given points
		/// @param      points - points to publish
		/// @return     void
		/// @author     Stephanie Meyer
		//========================================
		void publish_points(std::vector<pointcloud_utils::pointstruct>& points, std_msgs::msg::Header header)
		{
			visualization_msgs::msg::MarkerArray markers;
			visualization_msgs::msg::Marker marker;
		
		    //Leader point
			visualization_msgs::msg::Marker cloud_marker;
			cloud_marker.points.clear();
			cloud_marker.header = header;
			cloud_marker.ns = "cloud";
			cloud_marker.action = visualization_msgs::msg::Marker::ADD;
			cloud_marker.type = visualization_msgs::msg::Marker::POINTS;
		
			cloud_marker.scale.x = 0.05; //size of point
			cloud_marker.scale.y = 0.05;
		
			cloud_marker.color.r = 0.0;
			cloud_marker.color.g = 0.0;
			cloud_marker.color.b = 1.0;
			cloud_marker.color.a = 1.0;
		
			uint count = 0;
			for (pointcloud_utils::pointstruct pt : points)
			{
				geometry_msgs::msg::Point pt2;
				pt2.x = pt.x;
				pt2.y = pt.y;
				pt2.z = pt.z;
		
				cloud_marker.points.push_back(pt2);
			}
		
			markers.markers.push_back(cloud_marker);
		
			marker_pub->publish(markers);
		}
		
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
		{
			std::cout << "\n\nNew cloud recieved.\n";
			
			//Send cloud into the converter:
			std::vector<pointcloud_utils::pointstruct> cloud;
			sensor_msgs::msg::LaserScan scan;
			converter->convertCloud(msg, cloud, scan);
		
			if (visualize_3D)
			{
				publish_points(cloud, msg->header);
			}
		
			scan_pub->publish(scan);
		}
};


int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserscanNode>());
    rclcpp::shutdown();
	return(0);
}