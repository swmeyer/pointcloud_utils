/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 8 Sept 2020
 * Brief: implements a class that does a plane fit to the points in the given window, and reports the motion of the fitted plane
 * File: plane_parser_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud_2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <math.h>

#include <pointcloud_utils/pointcloud_utils.hpp>
#include "pointcloud_utils/processing/plane_parser.hpp"
#include "pointcloud_utils/processing/plane_parser_impl.hpp"
// --------------------------

class PlaneParserNode : public rclcpp::Node
{
	public:
		PlaneParserNode() :
			Node("plane_parser_node")
		{
			//main content

			//Declare Parameters
			this->declare_parameter<std::string>("cloud_topic_in", "/velodyne_points");
			this->declare_parameter<std::string>("cloud_topic_out", "/filtered_points");
			this->declare_parameter<std::string>("visualization_topic", "/markers");
		
			this->declare_parameter<bool>("print_plane_states", false);
			this->declare_parameter<bool>("print_plane_parameters", false);
		
			this->declare_parameter<std::string>("covariance_solution_type", "goodness_of_fit_error");
			this->declare_parameter<std::string>("plane_fit_type", "svd");
			this->declare_parameter<std::string>("angle_solution_type", "euler_angles");
			
			this->declare_parameter<bool>("use_point_track_method", false);
		
			this->declare_parameter<double>("plane_x_min", -10);
			this->declare_parameter<double>("plane_x_max", 10);
			this->declare_parameter<double>("plane_y_min", -10);
			this->declare_parameter<double>("plane_y_max", 10);
			this->declare_parameter<double>("plane_z_min", -10);
			this->declare_parameter<double>("plane_z_max", 10);
		
			this->declare_parameter<std::string>("plane_direction", "x");
		

			//Get Parameters
			std::string lidar_sub_topic, lidar_pub_topic;
			this->get_parameter<std::string>("cloud_topic_in", lidar_sub_topic);
			this->get_parameter<std::string>("cloud_topic_out", lidar_pub_topic);
		
			std::string marker_pub_topic;
			this->get_parameter<std::string>("visualization_topic", marker_pub_topic);
		
			this->get_parameter<bool>("print_plane_states", print_plane_states);
			this->get_parameter<bool>("print_plane_parameters", print_plane_parameters);
		
			std::string plane_fit_type, covariance_solution_type, angle_solution_type;
			this->get_parameter<std::string>("covariance_solution_type", covariance_solution_type);
			this->get_parameter<std::string>("plane_fit_type", plane_fit_type);
			this->get_parameter<std::string>("angle_solution_type", angle_solution_type);
			
			settings.plane_fit_type = pointcloud_utils::plane_parser_utils::convertPlaneFitType(plane_fit_type);
    		settings.covariance_type = pointcloud_utils::plane_parser_utils::convertCovarianceType(covariance_solution_type);
			settings.angle_solution_type = pointcloud_utils::plane_parser_utils::convertAngleSolutionType(angle_solution_type);
		
			this->get_parameter<bool>("use_point_track_method", settings.use_point_track_method);
		
			this->get_parameter<double>("plane_x_min", search_window.x_min);
			this->get_parameter<double>("plane_x_max", search_window.x_max);
			this->get_parameter<double>("plane_y_min", search_window.y_min);
			this->get_parameter<double>("plane_y_max", search_window.y_max);
			this->get_parameter<double>("plane_z_min", search_window.z_min);
			this->get_parameter<double>("plane_z_max", search_window.z_max);
		
			std::string plane_direction_string;
			this->get_parameter<std::string>("plane_direction", plane_direction_string);
		
			if (plane_direction_string == "X" || plane_direction_string == "x")
			{
				plane_direction = pointcloud_utils::PlaneParser::pointValueIndex::X;
			} else if (plane_direction_string == "Y" || plane_direction_string == "y")
			{
				plane_direction = pointcloud_utils::PlaneParser::pointValueIndex::Y;
			} else if (plane_direction_string == "Z" || plane_direction_string == "z")
			{
				plane_direction = pointcloud_utils::PlaneParser::pointValueIndex::Z;
			}
		
			this->get_parameter<bool>("iterate_plane_fit", settings.iterate_plane_fit);
			this->get_parameter<int>("max_plane_fit_iterations", settings.max_iterations);
			this->get_parameter<float>("outlier_point_tolerance", settings.outlier_tolerance);
			this->get_parameter<bool>("visualize_plane", visualize_plane);
			this->get_parameter<bool>("track_plane_states_over_time", settings.continue_from_last_plane);
		
			this->get_parameter<int>("min_points_to_fit", settings.min_points_to_fit);
    		this->get_parameter<bool>("report_offsets_at_origin", settings.report_offsets_at_origin);
			
			plane_parser = new pointcloud_utils::PlaneParser<pointcloud_utils::pointstruct>(settings);
			
		
			// this->get_parameter<int>("intensity_min", intensity_min);
			// this->get_parameter<int>("intensity_max", intensity_max);
		
			if (print_plane_states)
			{
		
				std::cout << "time, ";
				std::cout << "plane_x, plane_y, plane_z, plane_roll, plane_pitch, plane_yaw, plane_x_vel, plane_y_vel, plane_z_vel, plane_roll_vel, plane_pitch_vel, plane_yaw_vel, ";
			
				std::cout << "\n";
			}
			
		
			lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_pub_topic, 1);
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_pub_topic, 1);
			
			lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_sub_topic, 1, std::bind(&PlaneParserNode::pointCloudCallback, this, std::placeholders::_1));
		}

		~PlaneParserNode()
		{
			delete plane_parser;
		}

	private:
		//Variables
		bool print_plane_states;
		bool print_plane_parameters;
		bool visualize_plane;
		pointcloud_utils::PlaneParser::pointValueIndex plane_direction;
		
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;

		std_msgs::msg::Header header;
		std::vector<sensor_msgs::msg::PointField> fields;
		uint32_t point_step;
		
		
		pointcloud_utils::PlaneParser::Settings settings;
		pointcloud_utils::SearchWindow search_window;
		pointcloud_utils::PlaneParser<pointcloud_utils::pointstruct>* plane_parser;


		//Methods

		/**
		 * @Function 	visualizePlane
		 * @Param 		plane_coefficients - the a/d, b/d, c/d coefficients of the plane equation
		 * @Param 		id - plane ID number
		 * @Param 		solve_for - point value to find on plane 
		 * @Param 		min_1 - box point 1 value 1
		 * @Param 		max_1 - box point 2 value 1
		 * @Param 		min_2 - box point 1 value 2
		 * @Param 		max_2 - box point 2 value 2 - together these two points define the box diagonal
		 * @Return 		void
		 * @Brief 		Publishes the given plane as ros visualization lines
		 */
		void visualizePlane(const Eigen::Vector3f& plane_coefficients, const int id, const pointcloud_utils::PlaneParser::pointValueIndex solve_for, 
							const float min_1, const float max_1, const float min_2, const float max_2)
		{
			//std::cout << "Visualizing plane!\n";
			//std::cout << "Solving for: " << solve_for << "\n";
			visualization_msgs::msg::MarkerArray markers;
			visualization_msgs::msg::Marker marker;
			marker.header = header;
			marker.ns = "plane";
			marker.id = id;
			marker.type = visualization_msgs::msg::Marker::LINE_LIST;
			marker.scale.x = 0.01; //line thickness
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
		
			//add endpoint pairs to the marker.points
			geometry_msgs::msg::Point pt1;
			geometry_msgs::msg::Point pt2;
			geometry_msgs::msg::Point pt3;
			geometry_msgs::msg::Point pt4;
		
			switch (solve_for)
			{
				case (pointcloud_utils::PlaneParser::pointValueIndex::X):
				{
					//b/d y + c/d z -1 = -a/d x --> x = -(b/d * d/a) y - (c/d * d/a)z + d/a
					
					float y1 = min_1;
					float z1 = min_2;
					float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
					float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
					float d_prime = (1/plane_coefficients[0]);
					float x1 = b_prime * y1 + c_prime * z1 + d_prime;
					pt1.x = x1;
					pt1.y = y1;
					pt1.z = z1;
				
					float y2 = min_1;
					float z2 = max_2;
					float x2 = b_prime * y2 + c_prime * z2 + d_prime;
					pt2.x = x2;
					pt2.y = y2;
					pt2.z = z2;
				
					float y3 = max_1;
					float z3 = max_2;
					float x3 = b_prime * y3 + c_prime * z3 + d_prime;
					pt3.x = x3;
					pt3.y = y3;
					pt3.z = z3;
				
					float y4 = max_1;
					float z4 = min_2;
					float x4 = b_prime * y4 + c_prime * z4 + d_prime;
					pt4.x = x4;
					pt4.y = y4;
					pt4.z = z4;
					break;
				}
				case (pointcloud_utils::PlaneParser::pointValueIndex::Y):
				{
					//a/d x + c/d z -1 = -b/d y --> y = -(a/d * d/b) y - (c/d * d/b)z + d/b
					
					float x1 = min_1;
					float z1 = min_2;
					float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
					float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
					float d_prime = (1/plane_coefficients[1]);
					float y1 = a_prime * x1 + c_prime * z1 + d_prime;
					pt1.x = x1;
					pt1.y = y1;
					pt1.z = z1;
				
					float x2 = min_1;
					float z2 = max_2;
					float y2 = a_prime * x2 + c_prime * z2 + d_prime;
					pt2.x = x2;
					pt2.y = y2;
					pt2.z = z2;
				
					float x3 = max_1;
					float z3 = max_2;
					float y3 = a_prime * x3 + c_prime * z3 + d_prime;
					pt3.x = x3;
					pt3.y = y3;
					pt3.z = z3;
				
					float x4 = max_1;
					float z4 = min_2;
					float y4 = a_prime * x4 + c_prime * z4 + d_prime;
					pt4.x = x4;
					pt4.y = y4;
					pt4.z = z4;
					break;
				}
				case (pointcloud_utils::PlaneParser::pointValueIndex::Z):
				{
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
					break;
				}
				default:
				{
					//TODO: what do do here? should never happen
				}
			}
		
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
			marker_pub->publish(markers);
		}
		
		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
		{
			//std::cout << "Receiving new point cloud!\n";
			header = msg->header;
			fields = msg->fields;
			point_step = msg->point_step;
		
			std::vector<pointcloud_utils::pointstruct> cloud;
			sensor_msgs::msg::PointCloud2 filtered_points;
			pointcloud_utils::PlaneParser::PlaneParameters plane_parameters;
			pointcloud_utils::PlaneParser::States plane_states;
		
		
			plane_parser->parsePlane(msg, cloud, filtered_points, plane_parameters, plane_states, search_window);
			
		
			//std::cout << "Points in parsed cloud " << cloud_parsed.size() << "\n";
		
			//publishPoints(cloud_parsed);
		
			//TODO: add quaternion output?
		
			//publishPoints(cloud);
		
			if (print_plane_states)
			{
				//std::cout << "Cab states: \n";
				std::cout << msg->header.stamp << ", ";
				std::cout << plane_states.x << ", " << plane_states.y << ", " << plane_states.z << ", ";
				std::cout << plane_states.roll << ", " << plane_states.pitch << ", " << plane_states.yaw << ", ";
				std::cout << plane_states.x_vel << ", " << plane_states.y_vel << ", " << plane_states.z_vel << ", ";
				std::cout << plane_states.roll_vel << ", " << plane_states.pitch_vel << ", " << plane_states.yaw_vel << ", ";
		
				std::cout << "\n";
			}
			
		
			Eigen::Vector3f plane_coefficients;
			plane_coefficients << plane_parameters.a_d, plane_parameters.b_d, plane_parameters.c_d;
		
			if (print_plane_parameters)
			{
				std::cout << plane_coefficients(0) << ", " << plane_coefficients(1) << ", " << plane_coefficients(2) << "\n";
			}
		
			//visualize cloud!
			if (visualize_plane)
			{
		
				switch(plane_direction)
				{
					case(pointcloud_utils::PlaneParser::pointValueIndex::X):
					{
						visualizePlane(plane_coefficients, 0, plane_direction, search_window.y_min, search_window.y_max, search_window.z_min, search_window.z_max);
						break;
					}
					case(pointcloud_utils::PlaneParser::pointValueIndex::Y):
					{
						visualizePlane(plane_coefficients, 0, plane_direction, search_window.x_min, search_window.x_max, search_window.z_min, search_window.z_max);
						break;
					}
					case(pointcloud_utils::PlaneParser::pointValueIndex::Z):
					{
						visualizePlane(plane_coefficients, 0, plane_direction, search_window.x_min, search_window.x_max, search_window.y_min, search_window.y_max);
						break;
					}
					default:
					{
						//Nothing to do! shouldn't get here
					}
				}
			}
		}
};