/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 5 Feb 2020
 * Brief: publishes a set of distance markers in the form of lines parallel to the y-axis, at increasing values of x
 * File: distance_marker_publisher.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
// --------------------------

class DistanceMarkerPublisherNode : public rclcpp::Node
{
	public:
		DistanceMarkerPublisherNode() :
			Node("distance_marker_publisher_node")
		{
			//main content

			//Declare parameters
			this->declare_parameter<std::string>("publish_topic", "/distance_marker");
			this->declare_parameter<int>("publish_rate", 1);
			this->declare_parameter<double>("distance_interval", 10);
			this->declare_parameter<int>("number_of_markers", 3);
			this->declare_parameter<std::string>("publish_frame", "base_link");
			this->declare_parameter<double>("marker_width", 10);
		

			//Get parameters

			std::string marker_topic;
			this->get_parameter<std::string>("publish_topic", marker_topic);
			this->get_parameter<int>("publish_rate", rate);
			this->get_parameter<double>("distance_interval", interval);
			this->get_parameter<int>("number_of_markers", number_of_markers);
			this->get_parameter<std::string>("publish_frame", frame_id);
			this->get_parameter<double>("marker_width", marker_width);
		
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 1);
		
			createMarkers();
		
			publishMarkers();

		}

		~DistanceMarkerPublisherNode()
		{

		}

	private:

		//Variables
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; 				 // publisher for the distance markers
		visualization_msgs::msg::MarkerArray markers; // message to hold all the distance markers
		std::string frame_id; 					 // name of transform frame to publish on
		int rate; 								 // rate to publish at

		double interval;						 // [m] space between distance bars
		int number_of_markers;				 	 // number of distane bars to publish
		double marker_width; 					 // [m] width of marker lines, centered at x origin


		//Methods
		/**
		 * @Function 	createMarkers
		 * @Param 		none
		 * @Return 		void
		 * @Brief 		generate the distance marker lines
		 */
		void createMarkers()
		{
			//Distance Marker Lines
			visualization_msgs::msg::Marker distance_lines;
			distance_lines.points.clear();
			distance_lines.header.frame_id = frame_id;
			distance_lines.ns = "distance_markers";
			distance_lines.action = visualization_msgs::msg::Marker::ADD;
			distance_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
		
			distance_lines.scale.x = 0.1; //size of lines
		
			distance_lines.color.r = 0.0;
			distance_lines.color.g = 0.5;
			distance_lines.color.b = 0.5;
			distance_lines.color.a = 1.0;
		
		
			//Distance Marker Text Tags
			visualization_msgs::msg::Marker tags;
			tags.ns = "distance_tags";
			tags.header.frame_id = distance_lines.header.frame_id;
			tags.action = visualization_msgs::msg::Marker::ADD;
			tags.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		
			tags.scale.z = 1.5; //Font size
		
			tags.color.r = 0.3;
			tags.color.g = 0.0;
			tags.color.b = 0.7;
			tags.color.a = 1.0;
		
		
			double y_min = - (marker_width / 2);
			double y_max =   (marker_width / 2);
			double x;
		
			for (int i = 1; i < number_of_markers; i++)
			{
				x = i * interval;
				
				geometry_msgs::msg::Point pt;
				pt.x = x + 0.5;
				pt.y = y_min - 0.5;
				pt.z = 0.5;
				tags.pose.position = pt;
				tags.id = i;
				std::stringstream text;
				text << x;
				tags.text = text.str();
				markers.markers.push_back(tags);
		
				
				pt.x = x;
				pt.y = y_min;
				pt.z = 0.0;
				distance_lines.points.push_back(pt);
				pt.y = y_max;
				distance_lines.points.push_back(pt);
			}
		
			markers.markers.push_back(distance_lines);
			// std::cout << "Points: " << markers.points.size() << "\n";
		}
		
		/**
		 * @Function 	publishMarkers
		 * @Param 		none
		 * @Return 		void
		 * @Brief 		publish the distance markers at a regular interval
		 */
		void publishMarkers()
		{
			rclcpp::Rate pub_rate(rate);
			while (rclcpp::ok())
			{
				//publish markers
				marker_pub->publish(markers);
				pub_rate.sleep();
			}
		}
};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceMarkerPublisherNode>());
    rclcpp::shutdown();
	return(0);
}