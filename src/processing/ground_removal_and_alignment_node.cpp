/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2020
 * Brief: implementation of a class that detects and prcesses ground. Functions include removal or 
 *        segmentation of ground points and alignment of scan to ground
 * File: ground_removal_and_alignment_node.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/processing/ground_processor.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
// --------------------------

// --------------------------
ros::Publisher lidar_pub;
ros::Publisher ground_pub;
ros::Publisher nonground_pub;
ros::Publisher marker_pub;

pointcloud_utils::GroundProcessor* ground_processor;
pointcloud_utils::GroundProcessor::Settings settings;

std_msgs::Header header;

bool visualize_plane;

// --------------------------

/**
 * @Function 	visualizePlane
 * @Param 		plane_coefficients - the a/d, b/d, c/d coefficients of the plane equation
 * @Param 		id - plane ID number
 * @Param 		min_1 - box point 1 value 1
 * @Param 		max_1 - box point 2 value 1
 * @Param 		min_2 - box point 1 value 2
 * @Param 		max_2 - box point 2 value 2 - together these two points define the box diagonal
 * @Return 		void
 * @Brief 		Publishes the given plane as ros visualization lines
 */
void visualizePlane(const Eigen::Vector3f& plane_coefficients, const int id,
					const float min_1, const float max_1, const float min_2, const float max_2)
{
	//std::cout << "Visualizing plane!\n";
	//std::cout << "Solving for: " << solve_for << "\n";
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;
	marker.header = header;
	marker.ns = "plane";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.scale.x = 0.01; //line thickness
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	//add endpoint pairs to the marker.points
	geometry_msgs::Point pt1;
	geometry_msgs::Point pt2;
	geometry_msgs::Point pt3;
	geometry_msgs::Point pt4;

	//Assume a z-aligned plane:
	//a/d x + b/d y -1 = -c/d z --> z = -(a/d * d/c) y - (b/d * d/c)y + d/c
	//std::cout << "Solving for z at " << min_1 << ", " << min_2 << "\n";
	float x1 = min_1;
	float y1 = min_2;
	float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
	float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
	float d_prime = (1/plane_coefficients[2]);
	float z1 = a_prime * x1 + b_prime * y1 + d_prime;
	pt1.x = x1;
	pt1.y = y1;
	pt1.z = z1;
	
	float x2 = min_1;
	float y2 = max_2;
	float z2 = a_prime * x2 + b_prime * y2 + d_prime;
	pt2.x = x2;
	pt2.y = y2;
	pt2.z = z2;
	
	float x3 = max_1;
	float y3 = max_2;
	float z3 = a_prime * x3 + b_prime * y3 + d_prime;
	pt3.x = x3;
	pt3.y = y3;
	pt3.z = z3;
	
	float x4 = max_1;
	float y4 = min_2;
	float z4 = a_prime * x4 + b_prime * y4 + d_prime;
	pt4.x = x4;
	pt4.y = y4;
	pt4.z = z4;

	//Line 1-2
	marker.points.push_back(pt1);
	marker.points.push_back(pt2);

	//Line 2-3
	marker.points.push_back(pt2);
	marker.points.push_back(pt3);

	//Line 3-4
	marker.points.push_back(pt3);
	marker.points.push_back(pt4);

	//Line 4-1
	marker.points.push_back(pt4);
	marker.points.push_back(pt1);


	markers.markers.push_back(marker);

	//publish the plane
	marker_pub.publish(markers);
}


/**
 * @Function 	pointCloudCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	header = msg->header;

	sensor_msgs::PointCloud2 aligned_cloud;
	sensor_msgs::PointCloud2 ground_cloud;
	sensor_msgs::PointCloud2 nonground_cloud;
	std::vector<pointcloud_utils::pointstruct> point_vector;
	std::vector<pointcloud_utils::pointstruct> ground_vector;
	std::vector<pointcloud_utils::pointstruct> nonground_vector;

	ground_processor->updateCloud(msg, point_vector);

	//get plane params and visualize
	pointcloud_utils::PlaneParser::PlaneParameters plane_parameters;
	pointcloud_utils::PlaneParser::States plane_states;
	ground_processor->returnPlaneDescriptors(plane_parameters, plane_states);

	Eigen::Vector3f plane_coefficients;
	plane_coefficients << plane_parameters.a_d, plane_parameters.b_d, plane_parameters.c_d;

	//visualize cloud!
	if (visualize_plane)
	{
		visualizePlane(plane_coefficients, 0,
						settings.plane_search_window.x_min, settings.plane_search_window.x_max, 
						settings.plane_search_window.y_min, settings.plane_search_window.y_max);
	}

	ground_processor->alignToGround(aligned_cloud, point_vector);

	//std::cout << aligned_cloud.width << " points in aligned cloud vs " << msg->width << " points in original cloud.\n";

	ground_processor->separateGround(ground_cloud, nonground_cloud, ground_vector, nonground_vector);

	ground_processor->alignToGround(ground_cloud, ground_cloud, ground_vector);
	ground_processor->alignToGround(nonground_cloud, nonground_cloud, nonground_vector);

	//std::cout << "Ground points: " << ground_cloud.width << ", nonground points: " << nonground_cloud.width << "\n";


	lidar_pub.publish(aligned_cloud);
	ground_pub.publish(ground_cloud);
	nonground_pub.publish(nonground_cloud);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ground_processor");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string lidar_sub_topic, lidar_pub_topic, ground_pub_topic, nonground_pub_topic;
	n_.param<std::string>("cloud_topic_in", lidar_sub_topic, "/velodyne_points");
	n_.param<std::string>("aligned_topic_out", lidar_pub_topic, "/aligned_points");
	n_.param<std::string>("ground_topic_out", ground_pub_topic, "/ground_points");
	n_.param<std::string>("nonground_topic_out", nonground_pub_topic, "/nonground_points");
	n_.param<bool>("visualize_plane", visualize_plane, true);

	std::string marker_pub_topic;
	n_.param<std::string>("visualization_topic", marker_pub_topic, "/markers");

	n_.param<double>("plane_x_min", settings.plane_search_window.x_min, -10);
	n_.param<double>("plane_x_max", settings.plane_search_window.x_max, 10);
	n_.param<double>("plane_y_min", settings.plane_search_window.y_min, -10);
	n_.param<double>("plane_y_max", settings.plane_search_window.y_max, 10);
	n_.param<double>("plane_z_min", settings.plane_search_window.z_min, -10);
	n_.param<double>("plane_z_max", settings.plane_search_window.z_max, 10);

	n_.param<bool>("use_point_track_method", settings.plane_parser_settings.use_point_track_method, false);
	n_.param<bool>("iterate_plane_fit", settings.plane_parser_settings.iterate_plane_fit, false);
	n_.param<int>("max_plane_fit_iterations", settings.plane_parser_settings.max_iterations, 10 );
	n_.param<float>("outlier_point_tolerance", settings.plane_parser_settings.outlier_tolerance, 0.1);
	n_.param<int>("min_points_to_fit", settings.plane_parser_settings.min_points_to_fit, 20);
    n_.param<bool>("report_offsets_at_origin", settings.plane_parser_settings.report_offsets_at_origin, false);
    
    bool find_attitude_angles, find_euler_angles, find_simple_angles, find_quaternions;
    n_.param<bool>("find_attitude_angles", find_attitude_angles, true);
    n_.param<bool>("find_euler_angles", find_euler_angles, false);
    n_.param<bool>("find_simple_angles", find_simple_angles, false);
    n_.param<bool>("find_quaternions", find_quaternions, false);
	
	n_.param<float>("intensity_min", settings.intensity_min, 0);
	n_.param<float>("intensity_max", settings.intensity_max, 256);

    n_.param<std::string>("aligned_cloud_frame", settings.aligned_cloud_frame, "/ground");
    n_.param<float>("ground_point_tolerance", settings.point_to_plane_tolerance, 0.1);		

    if (find_attitude_angles)
    {
    	settings.plane_parser_settings.angle_solution_type = pointcloud_utils::PlaneParser::AngleSolutionType::ATTITUDE_ANGLES;
    } else if (find_euler_angles)
    {
    	settings.plane_parser_settings.angle_solution_type = pointcloud_utils::PlaneParser::AngleSolutionType::EULER_ANGLES;
    } else if (find_simple_angles)
    {
    	settings.plane_parser_settings.angle_solution_type = pointcloud_utils::PlaneParser::AngleSolutionType::SIMPLE_ANGLES;
    } else
    {
    	//Default to quaternions
    	settings.plane_parser_settings.angle_solution_type = pointcloud_utils::PlaneParser::AngleSolutionType::QUATERNIONS;
    }

    //TODO: set covariance settings for plane parser

	ground_processor = new pointcloud_utils::GroundProcessor(settings);

	lidar_pub = n.advertise<sensor_msgs::PointCloud2>(lidar_pub_topic, 1);
	ground_pub = n.advertise<sensor_msgs::PointCloud2>(ground_pub_topic, 1);
	nonground_pub = n.advertise<sensor_msgs::PointCloud2>(nonground_pub_topic, 1);
	marker_pub = n.advertise<visualization_msgs::MarkerArray>(marker_pub_topic, 1);
	
	ros::Subscriber lidar_sub = n.subscribe(lidar_sub_topic, 1, &pointCloudCallback);

	ros::spin();

	delete ground_processor;

	return 0;
}