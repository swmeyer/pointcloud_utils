/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Jun 2020
 * Brief: converts a 3D pointcloud into a 2D laserscan message
 * File: pointcloud_to_laserscan.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
// --------------------------

//typedef struct //Real VLP 16 struct
//{
//	float x;
//	float y;
//	float z;
//	float dummy;
//	float intensity;
//	unsigned short int ring;
//	unsigned short int dummy2;
//	float dummy3;
//	float dummy4;
//} pointstruct;

typedef struct //SWri airsim VLP 16 struct
{
	float x;
	float y;
	float z;
} pointstruct;

typedef struct
{
	float range;
	float vertical_angle;
	float azimuth;
} sphericalpoint;

// --------------------------
const float PI = 3.14;
ros::Publisher scan_pub;
float angle; //[rad] vertical angle at which to report the laser scan
float scan_height; //[m] height of laserscan origin in original frame
float position_tolerance; //[m] tolerance to accept a point into the laser scan
float resolution; //[rad] azimuthal increment for laser scan
int num_pts; //number of increments around a full-circle
std::string scan_frame;
ros::Publisher marker_pub; // publisher for visualization

bool visualize_3D; //set to true to visualize the converted 3D point coud, to verify poinstruct parsing
// --------------------------

/**
 * @function inTolerance
 * @brief    checks if the two given data are within tolerance of each other
 * @param    data1 		- first data to compare
 * @param 	 data2 		- second data to compare
 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
 * @return 	 bool - true if data1 and data2 are within tolerance of each other
 * 				  - false otherwise 
 */
bool inTolerance(float data1, float data2, float tolerance) //TODO: this could be an in-line function
{
	if ( abs(data1 - data2) <= tolerance )
	{
		return true;
	}

	return false;
}

//========================================
/// @fn         publish_points
/// @brief      publishes visualization for the given points
/// @param      points - points to publish
/// @return     void
/// @author     Stephanie Meyer
//========================================
void publish_points(std::vector<pointstruct>& points, std_msgs::Header header)
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
	for (pointstruct pt : points)
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
	//convert cloud to 2D and republish!
	//TODO: if there is a ring field, just use one of the rings as the laserscan

	//Set up scan message
	sensor_msgs::LaserScan scan;
	scan.header = msg->header;
	scan.header.frame_id = scan_frame;
	scan.angle_increment = resolution;
	scan.angle_min = 0;
	scan.angle_max = 2 * PI;
	scan.range_min = 0;
	scan.range_max = 500;

	//convert pointcloud
	std::vector<pointstruct> cloud;
	cloud.resize(msg->width);
	std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);
	std::cout << cloud.size() << " points.\n";

	//parse points into laser scan
	float z_target;
	float azimuth;
	float range;
	int increment_index;
	//std::vector<sphericalpoint> scan_points(num_pts, std::numeric_limits<float>::infinity() );
	scan.ranges.resize(num_pts, std::numeric_limits<float>::infinity());
	for (pointstruct pt : cloud)
	{
		//std::cout << "point\n";
		range = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2) + std::pow(pt.z, 2));
		if (angle == 0)
		{
			z_target = scan_height;
		} else
		{
			z_target = range * std::sin(angle) + scan_height;
		}
		
		if (inTolerance(z_target, pt.z, position_tolerance))
		{
			//std::cout << "Point in tolerance\n";
		    //range = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
			if (range != 0)
			{
	 			std::cout << "\nPoint: " << pt.x << ", " << pt.y << ", " << pt.z << "\n";
				std::cout << "Range: " << range << " flat range: " << std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2)) << "\n";
				//Add this point to the laser scan!!
				//azimuth = atan2(pt.y, pt.x);
				//std::cout << "cos: " << pt.x / std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2)) << "\n";
	 			azimuth = std::acos(pt.x / std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2)));
	 			if ( pt.y < 0)
	 			{
	 				azimuth = (2 * PI) - azimuth;
	 			}
	 			std::cout << "Azimuth: " << azimuth << "\n";
	 			increment_index = azimuth / resolution;
		
	 			//sphericalpoint = scan_pt;
	 			//scan_pt.range = range;
	 			//scan_pt.vertical_angle = angle;
	 			//scan_pt.azimuth = increment_index * 0.1;
		
				//This allows the points to be sorted by azimuth, which the laser scan expects
	 			//Note: there is probably a better way to store these points, so that we can just do a memcpy of ranges!
	 			//scan_points[increment_index] = scan_pt;
	 			std::cout << "Num pts: " << num_pts << ", vector size: " << scan.ranges.size() << "\n";
	 			std::cout << "Index: " << increment_index << "\n";
	 			if (increment_index >= num_pts || increment_index < 0)
	 			{
	 				continue; //out of bounds
	 			}
	 			scan.ranges[increment_index] = range;
			}
		} 
	}

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

	n_.param<float>("scan_angle", angle, 0.0);
	n_.param<float>("scan_height", scan_height, 0.0);
	n_.param<float>("position_tolerance", position_tolerance, 0.1);
	n_.param<float>("resolution", resolution, 0.01);
	n_.param<std::string>("scan_frame", scan_frame, "scan");
	n_.param<bool>("visualize_3D_scan", visualize_3D, false); //set to true to turn on visualization for verify poinstruct parsing

	//TODO: allow for limited horizonal view
	num_pts = (2 * PI) / resolution;
	
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cloud_markers", 1);
	
	scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_pub_topic, 1);
	ros::Subscriber cloud_sub = n.subscribe(cloud_sub_topic, 1, &cloudCallback);

	ros::spin();
	return(0);
}