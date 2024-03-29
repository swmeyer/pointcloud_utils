/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2020
 * Brief: implementation of a class that detects and prcesses ground. Functions include removal or 
 *        segmentation of ground points and alignment of scan to ground
 * File: ground_removal_and_alignment_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/processing/ground_processor.hpp>
#include <pointcloud_utils/processing/ground_processor_impl.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
// --------------------------

class GroundRemovalAndAlignmentNode : public rclcpp::Node
{
	public:
		GroundRemovalAndAlignmentNode() : 
			Node("ground_removal_and_alignment_node")
		{
			//main content

			RCLCPP_INFO(this->get_logger(), "Starting ground processor");

			//Declare Parameters
			this->declare_parameter<std::string>("cloud_topic_in", "/velodyne_points");
			this->declare_parameter<std::string>("aligned_topic_out", "/aligned_points");
			this->declare_parameter<std::string>("ground_topic_out", "/ground_points");
			this->declare_parameter<std::string>("nonground_topic_out", "/nonground_points");

			this->declare_parameter<bool>("visualize_plane", true);
			this->declare_parameter<bool>("align_to_ground", true);
			this->declare_parameter<bool>("separate_ground", true);
			this->declare_parameter<bool>("use_lumin2", false);
		
			this->declare_parameter<std::string>("visualization_topic", "/plane_markers");

			this->declare_parameter<double>("plane_x_min", -10);
			this->declare_parameter<double>("plane_x_max",  10);
			this->declare_parameter<double>("plane_y_min", -10);
			this->declare_parameter<double>("plane_y_max",  10);
			this->declare_parameter<double>("plane_z_min", -10);
			this->declare_parameter<double>("plane_z_max",  10);

			this->declare_parameter<int>("point_skip_num", 1);
			this->declare_parameter<bool>("invert_cloud_step_through", false);
			this->declare_parameter<int>("max_point_count", 5000);
		
			this->declare_parameter<bool>("iterate_plane_fit", false);
			this->declare_parameter<int>("max_plane_fit_iterations", 10);
			this->declare_parameter<float>("outlier_point_tolerance", 0.1);
			this->declare_parameter<int>("min_points_to_fit", 20);
    		this->declare_parameter<bool>("report_offsets_at_origin", false);
    		
    		this->declare_parameter<std::string>("covariance_solution_type", "goodness_of_fit_error");
			this->declare_parameter<std::string>("plane_fit_type", "svd");
			this->declare_parameter<std::string>("angle_solution_type", "euler_angles");
			
			this->declare_parameter<float>("intensity_min", 0);
			this->declare_parameter<float>("intensity_max", 256);
		
    		this->declare_parameter<std::string>("aligned_cloud_frame", "/ground");
    		this->declare_parameter<float>("ground_point_tolerance", 0.1);
    		this->declare_parameter<float>("pt_slope_threshold", 1.15);	

    		this->declare_parameter<float>("x_min", -100.0);
    		this->declare_parameter<float>("x_max", 100.0);
    		this->declare_parameter<float>("y_min", -100.0);
    		this->declare_parameter<float>("y_max", 100.0);
    		this->declare_parameter<float>("z_min", -10.0);
    		this->declare_parameter<float>("z_max", 10.0);

    		this->declare_parameter<float>("ego_x_min", -3.0);
    		this->declare_parameter<float>("ego_x_max", 3.0);
    		this->declare_parameter<float>("ego_y_min", -2.0);
    		this->declare_parameter<float>("ego_y_max", 2.0);
    		this->declare_parameter<float>("ego_z_min", 0.0);
    		this->declare_parameter<float>("ego_z_max", 4.0);

    		this->declare_parameter<float>("consecutive_height_tolerance", 1.0);
		

			//Get Parameters
			std::string lidar_sub_topic, lidar_pub_topic, ground_pub_topic, nonground_pub_topic;
			this->get_parameter<std::string>("cloud_topic_in", lidar_sub_topic);
			this->get_parameter<std::string>("aligned_topic_out", lidar_pub_topic);
			this->get_parameter<std::string>("ground_topic_out", ground_pub_topic);
			this->get_parameter<std::string>("nonground_topic_out", nonground_pub_topic);

			this->get_parameter<bool>("visualize_plane", visualize_plane);
			this->get_parameter<bool>("align_to_ground", align_to_ground);
			this->get_parameter<bool>("separate_ground", separate_ground);
			this->get_parameter<bool>("use_lumin2", this->use_lumin2);
		
			std::string marker_pub_topic;
			this->get_parameter<std::string>("visualization_topic", marker_pub_topic);

			this->get_parameter<double>("plane_x_min", settings.plane_search_window.x_min);
			this->get_parameter<double>("plane_x_max", settings.plane_search_window.x_max);
			this->get_parameter<double>("plane_y_min", settings.plane_search_window.y_min);
			this->get_parameter<double>("plane_y_max", settings.plane_search_window.y_max);
			this->get_parameter<double>("plane_z_min", settings.plane_search_window.z_min);
			this->get_parameter<double>("plane_z_max", settings.plane_search_window.z_max);

			this->get_parameter<int>("point_skip_num", settings.plane_parser_settings.point_skip_num);
			this->get_parameter<bool>("invert_cloud_step_through", settings.plane_parser_settings.invert_cloud_step_through);
			this->get_parameter<int>("max_point_count", settings.plane_parser_settings.max_point_count);
		
			this->get_parameter<bool>("iterate_plane_fit", settings.plane_parser_settings.iterate_plane_fit);
			this->get_parameter<int>("max_plane_fit_iterations", settings.plane_parser_settings.max_iterations);
			this->get_parameter<float>("outlier_point_tolerance", settings.plane_parser_settings.outlier_tolerance);
			this->get_parameter<int>("min_points_to_fit", settings.plane_parser_settings.min_points_to_fit);
    		this->get_parameter<bool>("report_offsets_at_origin", settings.plane_parser_settings.report_offsets_at_origin);
    		
    		bool find_attitude_angles, find_euler_angles, find_simple_angles, find_quaternions;
    		this->get_parameter<bool>("find_attitude_angles", find_attitude_angles);
    		this->get_parameter<bool>("find_euler_angles", find_euler_angles);
    		this->get_parameter<bool>("find_simple_angles", find_simple_angles);
    		this->get_parameter<bool>("find_quaternions", find_quaternions);

			std::string plane_fit_type, covariance_solution_type, angle_solution_type;
			this->get_parameter<std::string>("covariance_solution_type", covariance_solution_type);
			this->get_parameter<std::string>("plane_fit_type", plane_fit_type);
			this->get_parameter<std::string>("angle_solution_type", angle_solution_type);
			
			settings.plane_parser_settings.plane_fit_type = pointcloud_utils::plane_parser_utils::convertPlaneFitType(plane_fit_type);
    		settings.plane_parser_settings.covariance_type = pointcloud_utils::plane_parser_utils::convertCovarianceType(covariance_solution_type);
			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::convertAngleSolutionType(angle_solution_type);
		
			
			this->get_parameter<float>("intensity_min", settings.intensity_min);
			this->get_parameter<float>("intensity_max", settings.intensity_max);
		
    		this->get_parameter<std::string>("aligned_cloud_frame", settings.aligned_cloud_frame);
    		this->get_parameter<float>("ground_point_tolerance", settings.point_to_plane_tolerance);
    		this->get_parameter<float>("pt_slope_threshold", settings.pt_slope_threshold);		

    		this->get_parameter<float>("x_min", settings.x_min );
    		this->get_parameter<float>("x_max", settings.x_max );
    		this->get_parameter<float>("y_min", settings.y_min );
    		this->get_parameter<float>("y_max", settings.y_max );
    		this->get_parameter<float>("z_min", settings.z_min );
    		this->get_parameter<float>("z_max", settings.z_max );

    		this->get_parameter<float>("ego_x_min", settings.ego_x_min);
    		this->get_parameter<float>("ego_x_max", settings.ego_x_max);
    		this->get_parameter<float>("ego_y_min", settings.ego_y_min);
    		this->get_parameter<float>("ego_y_max", settings.ego_y_max);
    		this->get_parameter<float>("ego_z_min", settings.ego_z_min);
    		this->get_parameter<float>("ego_z_max", settings.ego_z_max);

    		this->get_parameter<float>("consecutive_height_tolerance", settings.plane_parser_settings.consecutive_height_tolerance);

		
    		if (find_attitude_angles)
    		{
    			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::AngleSolutionType::ATTITUDE_ANGLES;
    		} else if (find_euler_angles)
    		{
    			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::AngleSolutionType::EULER_ANGLES;
    		} else if (find_simple_angles)
    		{
    			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::AngleSolutionType::SIMPLE_ANGLES;
    		} else
    		{
    			//Default to quaternions
    			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::AngleSolutionType::QUATERNIONS;
    		}
		
    		//TODO: set covariance settings for plane parser
		
			ground_processor2 = new pointcloud_utils::GroundProcessor<pointcloud_utils::luminarPointstruct2>(settings);
			ground_processor = new pointcloud_utils::GroundProcessor<pointcloud_utils::luminarPointstruct>(settings);

			lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_pub_topic, rclcpp::SensorDataQoS());
			ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_pub_topic, rclcpp::SensorDataQoS());
			nonground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(nonground_pub_topic, rclcpp::SensorDataQoS());
			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_pub_topic, 1);
			
			lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_sub_topic, rclcpp::SensorDataQoS(), std::bind(&GroundRemovalAndAlignmentNode::pointCloudCallback, this, std::placeholders::_1));
		}

		~GroundRemovalAndAlignmentNode()
		{
			delete ground_processor;
			delete ground_processor2;
		}

	private:

		//Variables

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;

		pointcloud_utils::GroundProcessor<pointcloud_utils::luminarPointstruct>* ground_processor;
		pointcloud_utils::GroundProcessor<pointcloud_utils::luminarPointstruct2>* ground_processor2;
		pointcloud_utils::GroundProcessorSettings settings;
		
		std_msgs::msg::Header header;
		
		bool visualize_plane;
		bool align_to_ground;
		bool separate_ground;
		bool use_lumin2;


		//Methods

		void updateParams()
		{
			//Get Parameters
			std::string lidar_sub_topic, lidar_pub_topic, ground_pub_topic, nonground_pub_topic;
			this->get_parameter<std::string>("cloud_topic_in", lidar_sub_topic);
			this->get_parameter<std::string>("aligned_topic_out", lidar_pub_topic);
			this->get_parameter<std::string>("ground_topic_out", ground_pub_topic);
			this->get_parameter<std::string>("nonground_topic_out", nonground_pub_topic);

			this->get_parameter<bool>("visualize_plane", visualize_plane);
			this->get_parameter<bool>("align_to_ground", align_to_ground);
			this->get_parameter<bool>("separate_ground", separate_ground);
			this->get_parameter<bool>("use_lumin2", this->use_lumin2);
		
			std::string marker_pub_topic;
			this->get_parameter<std::string>("visualization_topic", marker_pub_topic);

			this->get_parameter<double>("plane_x_min", settings.plane_search_window.x_min);
			this->get_parameter<double>("plane_x_max", settings.plane_search_window.x_max);
			this->get_parameter<double>("plane_y_min", settings.plane_search_window.y_min);
			this->get_parameter<double>("plane_y_max", settings.plane_search_window.y_max);
			this->get_parameter<double>("plane_z_min", settings.plane_search_window.z_min);
			this->get_parameter<double>("plane_z_max", settings.plane_search_window.z_max);

			this->get_parameter<int>("point_skip_num", settings.plane_parser_settings.point_skip_num);
			this->get_parameter<bool>("invert_cloud_step_through", settings.plane_parser_settings.invert_cloud_step_through);
			this->get_parameter<int>("max_point_count", settings.plane_parser_settings.max_point_count);
		
			this->get_parameter<bool>("iterate_plane_fit", settings.plane_parser_settings.iterate_plane_fit);
			this->get_parameter<int>("max_plane_fit_iterations", settings.plane_parser_settings.max_iterations);
			this->get_parameter<float>("outlier_point_tolerance", settings.plane_parser_settings.outlier_tolerance);
			this->get_parameter<int>("min_points_to_fit", settings.plane_parser_settings.min_points_to_fit);
    		this->get_parameter<bool>("report_offsets_at_origin", settings.plane_parser_settings.report_offsets_at_origin);
    		
    		bool find_attitude_angles, find_euler_angles, find_simple_angles, find_quaternions;
    		this->get_parameter<bool>("find_attitude_angles", find_attitude_angles);
    		this->get_parameter<bool>("find_euler_angles", find_euler_angles);
    		this->get_parameter<bool>("find_simple_angles", find_simple_angles);
    		this->get_parameter<bool>("find_quaternions", find_quaternions);

			std::string plane_fit_type, covariance_solution_type, angle_solution_type;
			this->get_parameter<std::string>("covariance_solution_type", covariance_solution_type);
			this->get_parameter<std::string>("plane_fit_type", plane_fit_type);
			this->get_parameter<std::string>("angle_solution_type", angle_solution_type);
			
			settings.plane_parser_settings.plane_fit_type = pointcloud_utils::plane_parser_utils::convertPlaneFitType(plane_fit_type);
    		settings.plane_parser_settings.covariance_type = pointcloud_utils::plane_parser_utils::convertCovarianceType(covariance_solution_type);
			settings.plane_parser_settings.angle_solution_type = pointcloud_utils::plane_parser_utils::convertAngleSolutionType(angle_solution_type);
		
			
			this->get_parameter<float>("intensity_min", settings.intensity_min);
			this->get_parameter<float>("intensity_max", settings.intensity_max);
		
    		this->get_parameter<std::string>("aligned_cloud_frame", settings.aligned_cloud_frame);
    		this->get_parameter<float>("ground_point_tolerance", settings.point_to_plane_tolerance);
    		this->get_parameter<float>("pt_slope_threshold", settings.pt_slope_threshold);		

    		this->get_parameter<float>("x_min", settings.x_min );
    		this->get_parameter<float>("x_max", settings.x_max );
    		this->get_parameter<float>("y_min", settings.y_min );
    		this->get_parameter<float>("y_max", settings.y_max );
    		this->get_parameter<float>("z_min", settings.z_min );
    		this->get_parameter<float>("z_max", settings.z_max );

    		this->get_parameter<float>("ego_x_min", settings.ego_x_min);
    		this->get_parameter<float>("ego_x_max", settings.ego_x_max);
    		this->get_parameter<float>("ego_y_min", settings.ego_y_min);
    		this->get_parameter<float>("ego_y_max", settings.ego_y_max);
    		this->get_parameter<float>("ego_z_min", settings.ego_z_min);
    		this->get_parameter<float>("ego_z_max", settings.ego_z_max);

    		this->get_parameter<float>("consecutive_height_tolerance", settings.plane_parser_settings.consecutive_height_tolerance);

    		this->ground_processor2->updateSettings(this->settings);
    		this->ground_processor->updateSettings(this->settings);
		}

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
			marker_pub->publish(markers);
		}
		
		
		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			updateParams();
			
			header = msg->header;

		
			sensor_msgs::msg::PointCloud2 aligned_cloud;
			sensor_msgs::msg::PointCloud2 ground_cloud;
			sensor_msgs::msg::PointCloud2 nonground_cloud;
			std::vector<pointcloud_utils::luminarPointstruct> point_vector;
			std::vector<pointcloud_utils::luminarPointstruct> ground_vector;
			std::vector<pointcloud_utils::luminarPointstruct> nonground_vector;
			std::vector<pointcloud_utils::luminarPointstruct2> point_vector2;
			std::vector<pointcloud_utils::luminarPointstruct2> ground_vector2;
			std::vector<pointcloud_utils::luminarPointstruct2> nonground_vector2;

		
			if (this->use_lumin2)
			{
				ground_processor2->updateCloud(msg, point_vector2);
			} else
			{
				ground_processor->updateCloud(msg, point_vector);
			}
		
			//get plane params and visualize
			pointcloud_utils::plane_parser_utils::PlaneParameters plane_parameters;
			pointcloud_utils::plane_parser_utils::States plane_states;
			if (this->use_lumin2)
			{
				ground_processor2->returnPlaneDescriptors(plane_parameters, plane_states);
			} else
			{
				ground_processor->returnPlaneDescriptors(plane_parameters, plane_states);
			}
		
			Eigen::Vector3f plane_coefficients;
			plane_coefficients << plane_parameters.a_d, plane_parameters.b_d, plane_parameters.c_d;
		
			//visualize cloud!
			if (visualize_plane)
			{
				visualizePlane(plane_coefficients, 0,
							   settings.plane_search_window.x_min, settings.plane_search_window.x_max, 
							   settings.plane_search_window.y_min, settings.plane_search_window.y_max);
			}
			
			if (align_to_ground)
			{
				if (this->use_lumin2)
				{
					ground_processor2->alignToGround(aligned_cloud, point_vector2);
				} else
				{
					ground_processor->alignToGround(aligned_cloud, point_vector);
				}
		
				//std::cout << aligned_cloud.width << " points in aligned cloud vs " << msg->width << " points in original cloud.\n";

				lidar_pub->publish(aligned_cloud);
			}
			
			if (separate_ground)
			{
				if (this->use_lumin2)
				{

					ground_processor2->separateGround(ground_cloud, nonground_cloud, ground_vector2, nonground_vector2);
	
					//std::cout << "Ground points: " << ground_cloud.width << ", nonground points: " << nonground_cloud.width << "\n";
	
					if (align_to_ground)
					{
						ground_processor2->alignToGround(ground_cloud, ground_cloud, ground_vector2);
						ground_processor2->alignToGround(nonground_cloud, nonground_cloud, nonground_vector2);
					}
				} else
				{
					ground_processor->separateGround(ground_cloud, nonground_cloud, ground_vector, nonground_vector);
	
					//std::cout << "Ground points: " << ground_cloud.width << ", nonground points: " << nonground_cloud.width << "\n";
	
					if (align_to_ground)
					{
						ground_processor->alignToGround(ground_cloud, ground_cloud, ground_vector);
						ground_processor->alignToGround(nonground_cloud, nonground_cloud, nonground_vector);
					}
				}
	
				ground_pub->publish(ground_cloud);
				nonground_pub->publish(nonground_cloud);
			}
		}
};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundRemovalAndAlignmentNode>());
    rclcpp::shutdown();
	return(0);
}
