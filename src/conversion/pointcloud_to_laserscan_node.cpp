/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Jun 2020
 * Brief: implementation of a converter for a 3D pointcloud into a 2D laserscan message
 * File: pointcloud_to_laserscan_node.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <pointcloud_utils/conversion/pointcloud_to_laserscan.hpp>
#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

// --------------------------
ros::Publisher scan_pub;
std::string scan_frame;
ros::Publisher marker_pub; // publisher for visualization

pointcloud_utils::PointCloudToLaserScanConverter* converter;
pointcloud_utils::PointCloudToLaserScanConverter::Settings settings;

bool visualize_3D; //set to true to visualize the converted 3D point coud, to verify poinstruct parsing
// --------------------------

//========================================
/// @fn         publish_points
/// @brief      publishes visualization for the given points
/// @param      points - points to publish
/// @return     void
/// @author     Stephanie Meyer
//========================================
void publish_points(std::vector<pointcloud_utils::pointstruct>& points, std_msgs::Header header)
{
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;

    //Leader point
	visualization_msgs::Marker cloud_marker;
	cloud_marker.points.clear();
	cloud_marker.header = header;
	cloud_marker.ns = "cloud";
	cloud_marker.action = visualization_msgs::Marker::ADD;
	cloud_marker.type = visualization_msgs::Marker::POINTS;

	cloud_marker.scale.x = 0.05; //size of point
	cloud_marker.scale.y = 0.05;

	cloud_marker.color.r = 0.0;
	cloud_marker.color.g = 0.0;
	cloud_marker.color.b = 1.0;
	cloud_marker.color.a = 1.0;

	uint count = 0;
	for (pointcloud_utils::pointstruct pt : points)
	{
		geometry_msgs::Point pt2;
		pt2.x = pt.x;
		pt2.y = pt.y;
		pt2.z = pt.z;

		cloud_marker.points.push_back(pt2);
	}

	markers.markers.push_back(cloud_marker);

	marker_pub.publish(markers);
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "\n\nNew cloud recieved.\n";
	
	//Send cloud into the converter:
	std::vector<pointcloud_utils::pointstruct> cloud;
	sensor_msgs::LaserScan scan;
	converter->convertCloud(msg, cloud, scan);

	if (visualize_3D)
	{
		publish_points(cloud, msg->header);
	}

	scan_pub.publish(scan);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "laser_scan_conversion_node");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string cloud_sub_topic;
	std::string scan_pub_topic;
	n_.param<std::string>("cloud_sub_topic", cloud_sub_topic, "/cloud");
	n_.param<std::string>("scan_pub_topic",  scan_pub_topic,  "/scan");

	n_.param<float>("target_vertical_angle", settings.target_vertical_angle, 0.0);
	n_.param<float>("vertical_angle_tolerance", settings.vertical_angle_tolerance, 0.1);
	n_.param<bool>("filter_by_height", settings.filter_by_height, false);
	n_.param<float>("target_height", settings.target_height, 0.0);
	n_.param<float>("height_tolerance", settings.height_tolerance, 0.1);
	
	n_.param<bool>("use_fixed_resolution", settings.use_fixed_resolution, false);
    n_.param<float>("scan_resultion", settings.resolution, 0.001);

	n_.param<std::string>("scan_frame", scan_frame, "scan");
	settings.scan_frame = scan_frame;
	
	n_.param<bool>("visualize_3D_scan", visualize_3D, false); //set to true to turn on visualization for verify poinstruct parsing

	n_.param<float>("min_angle", settings.min_angle, -pointcloud_utils::PI);
	n_.param<float>("max_angle", settings.max_angle, pointcloud_utils::PI);

	n_.param<bool>("limit_front_distance", settings.limit_front_distance, false);
	n_.param<float>("front_distance_limit", settings.front_distance_limit, 100.0);
	
	converter = new pointcloud_utils::PointCloudToLaserScanConverter(settings);

	marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cloud_markers", 1);
	
	scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_pub_topic, 1);
	ros::Subscriber cloud_sub = n.subscribe(cloud_sub_topic, 1, &cloudCallback);

	ros::spin();

	delete converter;

	return(0);
}