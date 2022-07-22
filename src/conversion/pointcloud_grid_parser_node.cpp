/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 17 Feb 2020
 * Brief: a class to subscribe to point cloud messages and parse them into 2D grids
 * File: pointcloud_grid_parser.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// #include <rosbag/bag.h>
// #include <rosbag/view.h>

#include <Eigen/Dense>

// #include <dynamic_reconfigure/server.h>
// #include <pointcloud_utils/PointCloudUtilsConfig.h>

#include "pointcloud_utils/conversion/pointcloud_grid_parser.hpp"
#include "pointcloud_utils/conversion/pointcloud_grid_parser_impl.hpp"
// --------------------------

class PointCloudGridParserNode : public rclcpp::Node
{
	public:
		PointCloudGridParserNode() :
			Node("point_cloud_grid_parser_node")
		{
			RCLCPP_INFO(this->get_logger(), "Starting grid parser");
			//main content
		
			//std::cout << "\nNote: adjusting map to counter calibration: 3 0 1.5   0, 0.098796, -0.2469901, 0.9639685\n";
		
			grid_parser = new pointcloud_utils::PointCloudGridParser();
			
			//declare params
			this->declare_parameter<std::string>("cloud_topic", "/cloud");
			this->declare_parameter<std::string>("base_frame", "base_link");
			this->declare_parameter<std::string>("lidar_frame", "cloud");
			this->declare_parameter<std::string>("image_pub_topic", "/grid_image");
			this->declare_parameter<std::string>("costmap_pub_topic", "/map");

    		this->declare_parameter<bool>("use_luminar_pointstruct", true);
			

			//Map size params
			this->declare_parameter<double>("map_resolution", 0.5);
			this->declare_parameter<int>("map_width", 512);
			this->declare_parameter<int>("map_height", 512);
		
			this->declare_parameter<bool>("centered_x", false);
			this->declare_parameter<bool>("centered_y", true);
   		
    		this->declare_parameter<bool>("constant_map_size", false);
    		this->declare_parameter<bool>("constant_map_resolution", false);
    		this->declare_parameter<bool>("use_bounds", false);
    		this->declare_parameter<bool>("recalculate_dynamic_bounds", false);
   		
    		this->declare_parameter<double>("map_x_min", -10.0);
    		this->declare_parameter<double>("map_x_max", 10.0);
    		this->declare_parameter<double>("map_y_min", -10.0);
    		this->declare_parameter<double>("map_y_max", 10.0);
    		this->declare_parameter<double>("map_z_min", -10.0);
    		this->declare_parameter<double>("map_z_max", 10.0);


    		// Map value params
		
    		this->declare_parameter<bool>("binary_map", false);
    		this->declare_parameter<bool>("intensity_map", false);
    		this->declare_parameter<bool>("height_map", false);
    		this->declare_parameter<bool>("use_raytrace_to_clear_space", false);

    		this->declare_parameter<double>("binary_threshold", 1.0);

    		this->declare_parameter<double>("height_scale", 1.0);
    		this->declare_parameter<double>("intensity_scale", 1.0);


    		// Filter params
		
    		this->declare_parameter<double>("min_intensity", 0.0);
    		this->declare_parameter<double>("max_intensity", 256.0);

    		this->declare_parameter<double>("x_min", -10.0);
    		this->declare_parameter<double>("x_max", 10.0);
    		this->declare_parameter<double>("y_min", -10.0);
    		this->declare_parameter<double>("y_max", 10.0);
    		this->declare_parameter<double>("z_min", -10.0);
    		this->declare_parameter<double>("z_max", 10.0);

    		this->declare_parameter<int>("point_skip_num", 0);
		

			//get params
			std::string lidar_topic;
			std::string image_pub_topic;
			std::string costmap_pub_topic;
			this->get_parameter<std::string>("cloud_topic", lidar_topic);
			this->get_parameter<std::string>("base_frame", base_frame);
			this->get_parameter<std::string>("lidar_frame", lidar_frame);
			this->get_parameter<std::string>("image_pub_topic", image_pub_topic);
			this->get_parameter<std::string>("costmap_pub_topic", costmap_pub_topic);
		
    		this->get_parameter<bool>("use_luminar_pointstruct", settings.use_luminar_pointstruct);


			//Map size params
		
			this->get_parameter<double>("map_resolution", settings.resolution);
			std::cout << "Initial resolution: " << settings.resolution << "\n";
			this->get_parameter<int>("map_width", settings.map_width);
			this->get_parameter<int>("map_height", settings.map_height);
		
			this->get_parameter<bool>("centered_x", settings.centered_x);
			this->get_parameter<bool>("centered_y", settings.centered_y);
   		
    		this->get_parameter<bool>("constant_map_size", settings.const_size);
    		this->get_parameter<bool>("constant_map_resolution", settings.const_res);
    		this->get_parameter<bool>("use_bounds", settings.use_bounds);
    		this->get_parameter<bool>("recalculate_dynamic_bounds", settings.use_first);
   		
    		this->get_parameter<double>("map_x_min", settings.map_x_min);
    		this->get_parameter<double>("map_x_max", settings.map_x_max);
    		this->get_parameter<double>("map_y_min", settings.map_y_min);
    		this->get_parameter<double>("map_y_max", settings.map_y_max);
    		this->get_parameter<double>("map_z_min", settings.map_z_min);
    		this->get_parameter<double>("map_z_max", settings.map_z_max);


    		// Map value params
		
    		this->get_parameter<bool>("binary_map", settings.make_binary_map);
    		this->get_parameter<bool>("intensity_map", settings.make_intensity_map);
    		this->get_parameter<bool>("height_map", settings.make_height_map);
    		this->get_parameter<bool>("use_raytrace_to_clear_space", settings.use_raytrace_to_clear_space);

    		this->get_parameter<double>("binary_threshold", settings.binary_threshold);
			
    		this->get_parameter<double>("height_scale", settings.height_scale);
    		this->get_parameter<double>("intensity_scale", settings.intensity_scale);


			// Map filter params

    		this->get_parameter<double>("min_intensity",settings.min_intensity);
    		this->get_parameter<double>("max_intensity", settings.max_intensity);

    		this->get_parameter<double>("x_min", settings.x_min);
    		this->get_parameter<double>("x_max", settings.x_max);
    		this->get_parameter<double>("y_min", settings.y_min);
    		this->get_parameter<double>("y_max", settings.y_max);
    		this->get_parameter<double>("z_min", settings.z_min);
    		this->get_parameter<double>("z_max", settings.z_max);

    		this->get_parameter<int>("point_skip_num", settings.point_skip_num);



    		tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 1, std::bind(&PointCloudGridParserNode::tfCallback, this, std::placeholders::_1));
			
			// while (rclcpp::ok() && !transform_found)
    		// {
    		// 	rclcpp::Rate(10).sleep();
    		// 	std::cout << "waiting for transform.\n";
    		// 	rclcpp::spin_some(this);
    		// }
		
    		// if (!transform_found)
    		// {
    		// 	std::cout << "Failed to find transform. Shutting down due to external kill command.\n";
    		// 	return(0);
    		// }
		// 
    		// tf_sub->shutdown();
		
    		if (!grid_parser->init(settings, transform))
    		{
    			std::cout << "Failed to initialize the grid parser. Shutting down.\n";
    			delete(grid_parser);
    		} else
    		{				
				//subscribe/advertise
				map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_pub_topic, 1);
				image_pub = this->create_publisher<sensor_msgs::msg::Image>(image_pub_topic, 1);
				
				cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudGridParserNode::pointCloudCallback, this, std::placeholders::_1));
			}

			RCLCPP_INFO(this->get_logger(), "Grid parser started");
		}

		~PointCloudGridParserNode()
		{
			delete(grid_parser);
		}

	private:
		//Variables
		const double DEFAULT_RES = 1; //[m/cell] if const_res is false and conditions are undefined, use 1-1 res

		pointcloud_utils::PointCloudGridParser* grid_parser;
		pointcloud_utils::PointCloudGridParser::Settings settings;
		
		std_msgs::msg::Header header; //Header from most recent pointcloud message
		
		std::string base_frame; // common frame to transform points to
		std::string lidar_frame; // lidar transform frame
		geometry_msgs::msg::TransformStamped transform;
		bool transform_found = false; //marks whether static transform has been found
		
		
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
		rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;

		//Eigen::MatrixXf grid;
		std::vector<uint8_t> grid_bytes;
		std::vector<uint8_t> map_grid_bytes;


		//Methods

		/**
		 * @Function 	publishMap
		 * @Param 		grid - grid to publish as a map
		 * @Return 		void
		 * @Brief 		Publishes the given grid as an occupancygrid map
		 */
		void publishMap()
		{
			std::cout << "Publishing map\n";
			pointcloud_utils::PointCloudGridParser::Settings settings;
			grid_parser->getSettings(settings);
		
			//Make message
			nav_msgs::msg::OccupancyGrid map;
			map.header = header;
			map.info.map_load_time = header.stamp;
			map.info.resolution = settings.resolution;
			map.info.width 		= settings.map_width;
			map.info.height 	= settings.map_height;
		
			map.info.origin.position.x = settings.x_min;
			map.info.origin.position.y = settings.y_min;
			geometry_msgs::msg::Quaternion quat;
			//quat.x = 1;
			//quat.y = 0;
			//quat.z = 0;
			//quat.w = 0;
			quat.x = 0.7071068; //TODO: why??
			quat.y = 0.7071068;
			quat.z = 0;
			quat.w = 0;
			map.info.origin.orientation = quat;
		
			// map.info.origin.position.x = x_min + 3;
			// map.info.origin.position.y = y_min + 0;
			// map.info.origin.position.z = z_min + 1.5;
			// map.info.origin.orientation.x = 0;
			// map.info.origin.orientation.y = -0.098796;
			// map.info.origin.orientation.z =  0.2469901;
			// map.info.origin.orientation.w = 0.9639685;
		
		
			// note that this map is limited to 1 byte per cell
		
			int byte_size = settings.map_width * settings.map_height;
			map.data.resize(byte_size);
			memcpy(&(map.data[0]), &(map_grid_bytes[0]), byte_size);
		
			map_pub->publish(map);
		}
		
		/**
		 * @Function 	publishImage
		 * @Param 		grid - grid to publish as an image
		 * @Return 		void
		 * @Brief 		Publishes the given grid as a streaming image
		 */
		void publishImage()
		{
			std::cout << "Publishing image\n";
			pointcloud_utils::PointCloudGridParser::Settings settings;
			grid_parser->getSettings(settings);
			grid_parser->getGridBytes(grid_bytes);
		
			std::cout << "Grid bytes acquired.\n";
		
			//Make message
			sensor_msgs::msg::Image msg;
			msg.header = header;
			msg.height = settings.map_height;
			msg.width  = settings.map_width;
			msg.step   = settings.map_width * sizeof(grid_bytes[0]);
		
			msg.encoding = sensor_msgs::image_encodings::MONO8;
		
			int expected_cell_cout = settings.map_width * settings.map_height;
			if ((uint) expected_cell_cout != grid_bytes.size())
			{
				std::cout << "Warning: Expected and actual number of grid cells do not match: " << expected_cell_cout << " vs " << grid_bytes.size() << "\n";
			}
		
			std::cout << "Copying data into image msg\n";
			//int byte_size = settings.map_width * settings.map_height * sizeof(grid_bytes[0]);
			//int byte_size = grid_bytes.size() * sizeof(grid_bytes[0]); //This is dumb, right?
			int byte_size = grid_bytes.size();
			msg.data.resize(byte_size);
			std::cout << "Copy size: " << byte_size << "\n";
			memcpy(&(msg.data[0]), &(grid_bytes[0]), byte_size);
		
			std::cout << "Doing the publishing now:\n";
			image_pub->publish(msg);
		
			//TODO: publish a compressed image instead
			//TODO: publish offsets?
		}

		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			// RCLCPP_INFO(this->get_logger(), "Recieving new point cloud");
			header = msg->header;
			if (header.stamp.sec == 0)
			{
				header.stamp = this->now();
			}
		
			header.frame_id = base_frame;
			// if (has_new_params)
			// {
			// 	//map_width = new_map_width;
			// 	//map_height = new_map_height;
			// 	resolution = new_resolution;
			// 	value_scale_min = new_value_scale_min;
			// 	value_scale_max = new_value_scale_max;
			// 	has_new_params = false;
			// 	first = true;
			// }
			// RCLCPP_INFO(this->get_logger(), "new cloud received.");
			if (settings.use_luminar_pointstruct)
			{
				grid_parser->updateCloud<pointcloud_utils::luminarPointstruct>(msg, grid_bytes, map_grid_bytes);
				// RCLCPP_INFO(this->get_logger(), "Just returned");
			} else
			{
				grid_parser->updateCloud<pointcloud_utils::pointstruct>(msg, grid_bytes, map_grid_bytes);
			}
			// RCLCPP_INFO(this->get_logger(), "Grid updated.");
		
			publishImage();
			// RCLCPP_INFO(this->get_logger(), "Cycle done");
			
			//publishMap();
		}
		
		//========================================
		// @fn 		tfCallback
		// @param 	msg incoming tfs
		// @return 	void
		// @brief 	reacts to incoming tf messages
		//========================================
		void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
		{
			if (base_frame == lidar_frame)
		    {
		    	//Identity transform
		    	transform.header.frame_id = base_frame;
		    	transform.child_frame_id = lidar_frame;
		    	transform.transform.rotation.w = 1;
		    	transform_found = true;
		    	return;
		    }
			for (geometry_msgs::msg::TransformStamped tf : msg->transforms)
			{
				std::cout << "target parent: " << base_frame << ", target child: " << lidar_frame << "\n";
				std::cout << "Current parent: " << tf.header.frame_id << ", current child: " << tf.child_frame_id << "\n";
				if (tf.header.frame_id == base_frame && tf.child_frame_id == lidar_frame)
				{
					transform = tf;
					transform_found = true;
					return;
				}
			}
		}
};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGridParserNode>());
    rclcpp::shutdown();
	return(0);
}