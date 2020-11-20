/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 19 Nov 2020
 * Brief: publishes a set planar points
 * File: plane_points_generator.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
// --------------------------

// --------------------------
ros::Publisher markerPub; 				 // publisher for the distance markers
visualization_msgs::MarkerArray markers; // message to hold all the distance markers
std::string frame_id; 					 // name of transform frame to publish on
int rate; 								 // rate to publish at

double interval;						 // [m] space between distance bars
int number_of_markers;				 	 // number of distane bars to publish
double marker_width; 					 // [m] width of marker lines, centered at x origin
// --------------------------

/**
 * @Function 	createMarkers
 * @Param 		none
 * @Return 		void
 * @Brief 		generate the distance marker lines
 */
void createMarkers()
{
	//Distance Marker Lines
	visualization_msgs::Marker distance_lines;
	distance_lines.points.clear();
	distance_lines.header.frame_id = frame_id;
	distance_lines.ns = "distance_markers";
	distance_lines.action = visualization_msgs::Marker::ADD;
	distance_lines.type = visualization_msgs::Marker::LINE_LIST;

	distance_lines.scale.x = 0.1; //size of lines

	distance_lines.color.r = 0.0;
	distance_lines.color.g = 0.5;
	distance_lines.color.b = 0.5;
	distance_lines.color.a = 1.0;


	//Distance Marker Text Tags
	visualization_msgs::Marker tags;
	tags.ns = "distance_tags";
	tags.header.frame_id = distance_lines.header.frame_id;
	tags.action = visualization_msgs::Marker::ADD;
	tags.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	tags.scale.z = 1.5; //Font size

	tags.color.r = 0.3;
	tags.color.g = 0.0;
	tags.color.b = 0.7;
	tags.color.a = 1.0;


	double y_min = - (marker_width / 2);
	double y_max =   (marker_width / 2);
	double x;

	for (uint i = 1; i < number_of_markers; i++)
	{
		x = i * interval;
		
		geometry_msgs::Point pt;
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
	ros::Rate pub_rate(rate);
	while (ros::ok())
	{
		//publish markers
		markerPub.publish(markers);
		pub_rate.sleep();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "distance_publisher");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string marker_topic;
	n_.param<std::string>("publish_topic", marker_topic, "/distance_marker");
	n_.param<int>("publish_rate", rate, 1);
	n_.param<double>("distance_interval", interval, 10);
	n_.param<int>("number_of_markers", number_of_markers, 3);
	n_.param<std::string>("publish_frame", frame_id, "base_link");
	n_.param<double>("marker_width", marker_width, 10);

	markerPub = n.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);

	createMarkers();

	publishMarkers();

	ros::Rate rate(1);

	ros::spin();
	return(0);
}