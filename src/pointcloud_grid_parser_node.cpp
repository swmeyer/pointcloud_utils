/** Author: Stephanie Meyer swmeyer16@gmail.com 17 Feb 2020
/* Brief: a class to subscribe to point cloud messages and parse them into 2D grids
/* File: pointcloud_grid_parser.cpp
*/

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <pointcloud_utils/PointCloudUtilsConfig.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include "pointcloud_utils/pointcloud_grid_parser.hpp"
#include "pointcloud_utils/pointcloud_grid_parser_impl.hpp"
// --------------------------

const double DEFAULT_RES = 1; //[m/cell] if const_res is false and conditions are undefined, use 1-1 res

pointcloud_utils::PointCloudGridParser* grid_parser;
pointcloud_utils::PointCloudGridParser::Settings settings;

std_msgs::Header header; //Header from most recent pointcloud message

std::string base_frame; // common frame to transform points to
std::string lidar_frame; // lidar transform frame
geometry_msgs::TransformStamped transform;
bool transform_found = false; //marks whether static transform has been found


ros::Publisher map_pub;
ros::Publisher image_pub;
//Eigen::MatrixXf grid;
std::vector<uint8_t> grid_bytes;
std::vector<uint8_t> map_grid_bytes;

// int new_map_width;
// int new_map_height;
// int new_resolution;
// int new_z_scale_min;
// int new_z_scale_max;
bool has_new_params = false;
// --------------------------


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
	nav_msgs::OccupancyGrid map;
	map.header = header;
	map.info.map_load_time = header.stamp;
	map.info.resolution = settings.resolution;
	map.info.width 		= settings.map_width;
	map.info.height 	= settings.map_height;

	map.info.origin.position.x = settings.x_min;
	map.info.origin.position.y = settings.y_min;
	geometry_msgs::Quaternion quat;
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

	map_pub.publish(map);
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

	//Make message
	sensor_msgs::Image msg;
	msg.header = header;
	msg.height = settings.map_height;
	msg.width  = settings.map_width;
	msg.step   = settings.map_width * sizeof(grid_bytes[0]);

	msg.encoding = sensor_msgs::image_encodings::MONO8;

	int expected_cell_cout = settings.map_width * settings.map_height;
	if (expected_cell_cout != grid_bytes.size())
	{
		std::cout << "Warning: Expected and actual number of grid cells do not match: " << expected_cell_cout << " vs " << grid_bytes.size() << "\n";
	}
	//int byte_size = settings.map_width * settings.map_height * sizeof(grid_bytes[0]);
	int byte_size = grid_bytes.size() * sizeof(grid_bytes[0]);
	msg.data.resize(byte_size);
	memcpy(&(msg.data[0]), &(grid_bytes[0]), byte_size);

	image_pub.publish(msg);

	//TODO: publish a compressed image instead
	//TODO: publish offsets?
}

void reconfigureCallback(pointcloud_utils::PointCloudUtilsConfig &config, uint32_t level)
{
	pointcloud_utils::PointCloudGridParser::Settings settings;
	grid_parser->getSettings(settings);

	// new_resolution = config.map_resolution;
	// new_z_scale_min = config.z_scale_min;
	// new_z_scale_max = config.z_scale_max;
	// new_map_height =  config.map_height;
	// new_map_width = config.map_width;
	// has_new_params = true;

	settings.z_scale_min = config.z_scale_min;
	settings.z_scale_max = config.z_scale_max;
	settings.centered_x = config.x_centered;
	settings.centered_y = config.y_centered;

	

	has_new_params = true;

	std::cout << "Warning: dynamic reconfigure of map size turned off, due to corrupted functionality\n";
	bool turn_off_resize = false;
	if (!turn_off_resize)
	{
		settings.resolution = config.map_resolution;
		settings.x_min = config.x_min;
		settings.x_max = config.x_max;
		settings.y_min = config.y_min;
		settings.y_max = config.y_max;
		settings.z_min = config.z_min;
		settings.z_max = config.z_max;
	}
	//std::cout << "Updating settings via dynamic reconfigure.\n";
	//grid_parser->setSettings(settings);
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
	if (header.stamp.toSec() == 0)
	{
		header.stamp = ros::Time::now();
	}
	// if (has_new_params)
	// {
	// 	//map_width = new_map_width;
	// 	//map_height = new_map_height;
	// 	resolution = new_resolution;
	// 	z_scale_min = new_z_scale_min;
	// 	z_scale_max = new_z_scale_max;
	// 	has_new_params = false;
	// 	first = true;
	// }
	std::cout << "new cloud received.\n";
	if (settings.use_shell_pointstruct)
	{
		grid_parser->updateCloud<pointcloud_utils::simplePointstruct>(msg, grid_bytes, map_grid_bytes);
	} else
	{
		grid_parser->updateCloud<pointcloud_utils::pointstruct>(msg, grid_bytes, map_grid_bytes);
	}
	std::cout << "Grid updated\n";

	publishImage();
	//publishMap();
}

//========================================
// @fn 		tfCallback
// @param 	msg incoming tfs
// @return 	void
// @brief 	reacts to incoming tf messages
//========================================
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
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
	for (geometry_msgs::TransformStamped tf : msg->transforms)
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

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pointcloud_grid_parser");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	//std::cout << "\nNote: adjusting map to counter calibration: 3 0 1.5   0, 0.098796, -0.2469901, 0.9639685\n";

	grid_parser = new pointcloud_utils::PointCloudGridParser();

	//get params
	std::string lidar_topic;
	std::string image_pub_topic;
	std::string costmap_pub_topic;
	n_.param<std::string>("cloud_topic", lidar_topic, "/cloud");
	n_.param<std::string>("base_frame", base_frame, "base_link");
	n_.param<std::string>("lidar_frame", lidar_frame, "cloud");
	n_.param<std::string>("image_pub_topic", image_pub_topic, "/grid_image");
	n_.param<std::string>("costmap_pub_topic", costmap_pub_topic, "/map");
	n_.param<double>("map_resolution", settings.resolution, 0.5);
	n_.param<int>("map_width", settings.map_width, 512);
	n_.param<int>("map_height", settings.map_height, 512);

	n_.param<bool>("centered_x", settings.centered_x, false);
	n_.param<bool>("centered_y", settings.centered_y, true);
   
    n_.param<bool>("constant_map_size", settings.const_size, false);
    n_.param<bool>("constant_map_resolution", settings.const_res, false);
    n_.param<bool>("use_bounds", settings.use_bounds, false);
    n_.param<bool>("recalculate_dynamic_bounds", settings.use_first, false);
    n_.param<bool>("use_raytrace_to_clear_space", settings.use_raytrace_to_clear_space, false);
   
    n_.param<double>("x_min", settings.x_min, -10);
    n_.param<double>("x_max", settings.x_max, 10);
    n_.param<double>("y_min", settings.y_min, -10);
    n_.param<double>("y_max", settings.y_max, 10);
    n_.param<double>("z_min", settings.z_min, -10);
    n_.param<double>("z_max", settings.z_max, 10);

    n_.param<double>("z_scale_max", settings.z_scale_max, 10);
    n_.param<double>("z_scale_min", settings.z_scale_min, 10);
    n_.param<bool>("binary_map", settings.make_binary_map, false);

    n_.param<bool>("use_shell_pointstruct", settings.use_shell_pointstruct, false);

    ros::Subscriber tf_sub = n.subscribe("/tf", 1, &tfCallback);

    while (ros::ok() && !transform_found)
    {
    	ros::Rate(10).sleep();
    	std::cout << "waiting for transform.\n";
    	ros::spinOnce();
    }

    if (!transform_found)
    {
    	std::cout << "Failed to find transform. Shutting down due to external kill command.\n";
    	return(0);
    }

    tf_sub.shutdown();

    if (!grid_parser->init(settings, transform))
    {
    	std::cout << "Failed to initialize the grid parser. Shutting down.\n";
    	delete(grid_parser);
    	return(1);
    }

    dynamic_reconfigure::Server<pointcloud_utils::PointCloudUtilsConfig> server;
  	dynamic_reconfigure::Server<pointcloud_utils::PointCloudUtilsConfig>::CallbackType f;

    f = boost::bind(&reconfigureCallback, _1, _2);
  	server.setCallback(f);

	//subscribe/advertise
	map_pub = n.advertise<nav_msgs::OccupancyGrid>(costmap_pub_topic, 1);
	image_pub = n.advertise<sensor_msgs::Image>(image_pub_topic, 1);
	ros::Subscriber cloud_sub = n.subscribe(lidar_topic, 1, &pointCloudCallback);

	ros::spin();

	delete(grid_parser);

	return(0);
}