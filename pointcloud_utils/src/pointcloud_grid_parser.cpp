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
// --------------------------

// --------------------------
enum mapRules
{
	FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES = 0,
	FIT_RES_DYNAMIC_BOUNDS,
	FIT_CELLS_DYNAMIC_BOUNDS,
	FIT_BOUNDS,
	FIT_CELLS_UNIT_RES,
	FIT_RES,
	FIT_CELLS,
	HOLD_ALL = 7
};

typedef struct
{
	float x;
	float y;
	float z;
	float dummy;
	float intensity;
	unsigned short int ring;
	unsigned short int dummy2;
	float dummy3;
	float dummy4;
} pointstruct;

std_msgs::Header header; //Header from most recent pointcloud message

std::string base_frame; // common frame to transform points to
std::string lidar_frame; // lidar transform frame
geometry_msgs::TransformStamped transform;
bool transform_found = false; //marks whether static transform has been found


ros::Publisher map_pub;
ros::Publisher image_pub;
Eigen::MatrixXf grid;
std::vector<uint8_t> grid_bytes;
std::vector<uint8_t> map_grid_bytes;
double resolution;			// [m/cell] grid resolution
const double DEFAULT_RES = 1; //[m/cell] if const_res is false and conditions are undefined, use 1-1 res
int map_width;				//num cells in width
int map_height; 			//num cells in height

bool centered_x;
bool centered_y;

int new_map_width;
int new_map_height;
int new_resolution;
int new_z_scale_min;
int new_z_scale_max;
bool has_new_params = false;

bool const_res;				//flag to freeze map resolution
bool const_size; 			//flag to limit map by cell num
bool use_bounds; 			//flag to use bounds on area of interest or not
bool use_first; 			//flag to control whether dynamic bounds are found once or every time

// Area of interest bounds:
double x_min;
double x_max;
double y_min;
double y_max;
double z_min;
double z_max;

double z_scale_max;
double z_scale_min; //scaling parameters for converting between data types
bool make_binary_map; //if true, occupied cells have value 255, unoccupied 0
// --------------------------

//========================================
/// @fn         transformToOutputFrame
/// @brief      handle transform to base frame
/// @param      input_msg - untransformed message
/// @param      output_msg - holder for all transformed messages
/// @return     bool - true if the transform is successful, else false
/// @details    Converts the given input to the common output frame
/// @author     Stephanie Meyer
//========================================
bool transformToOutputFrame(
    const sensor_msgs::PointCloud2::ConstPtr& input_msg,
    sensor_msgs::PointCloud2& output_msg
)
{
	tf2::doTransform (*input_msg, output_msg, transform);
	header.frame_id = output_msg.header.frame_id;
	return true;
}


/**
 * @Function 	publishMap
 * @Param 		grid - grid to publish as a map
 * @Return 		void
 * @Brief 		Publishes the given grid as an occupancygrid map
 */
void publishMap()
{
	//TODO: test

	//Make message
	nav_msgs::OccupancyGrid map;
	map.header = header;
	map.info.map_load_time = header.stamp;
	map.info.resolution = resolution;
	map.info.width = map_width;
	map.info.height = map_height;

	map.info.origin.position.x = x_min;
	map.info.origin.position.y = y_min;
	geometry_msgs::Quaternion quat;
	//quat.x = 1;
	//quat.y = 0;
	//quat.z = 0;
	//quat.w = 0;
	quat.x = 0.7071068;
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

	int byte_size = map_width * map_height;
	map.data.resize(byte_size);
	memcpy(&(map.data[0]), &(map_grid_bytes[0]),  byte_size);

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
	//Make message
	sensor_msgs::Image msg;
	msg.header = header;
	msg.height = map_height;
	msg.width = map_width;
	msg.step = map_width * sizeof(grid_bytes[0]);

	msg.encoding = sensor_msgs::image_encodings::MONO8;


	int byte_size = map_width * map_height * sizeof(grid_bytes[0]);
	msg.data.resize(byte_size);
	memcpy(&(msg.data[0]), &(grid_bytes[0]), byte_size);

	image_pub.publish(msg);

	//TODO: publish a compressed image instead
	//TODO: publish offsets?
}

/**
 * @Function 	parseGrid
 * @Param 		msg - incoming data message
 * @Param 		grid - grid to parse into
 * @Param 		initialize - flag to indicate to re-initialize the grid size or not
 * @Return 		void
 * @Brief 		Sorts the given point cloud into a 2D grid
 */
void parseGrid(std::vector<pointstruct>& cloud, Eigen::MatrixXf& grid, const bool initialize)
{
	//Determine map params --------------------------------------------------
	unsigned char rule = 0;
	rule |= const_res;
	rule = rule << 1;
	rule |= const_size;
	rule = rule << 1;
	rule |= use_bounds;

	if ((initialize && use_first) || (!use_first))
	{ //TODO: this num cells and resolution and bounds could be better handled by a quadtree

		//We'll have to go through the cloud twice, this first time to get grid size params
		for (pointstruct pt : cloud)
		{
			if (rule == mapRules::FIT_CELLS_DYNAMIC_BOUNDS || 
				 rule == mapRules::FIT_RES_DYNAMIC_BOUNDS ||
				 rule == mapRules::FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES)
			{
				x_min = (x_min > pt.x)?pt.x:x_min;
				y_min = (y_min > pt.y)?pt.y:y_min;
	
				x_max = (x_max < pt.x)?pt.x:x_max;
				y_max = (y_max < pt.y)?pt.y:y_max;

				z_max = (z_max < pt.z)?pt.z:z_max;
				z_min = (z_min > pt.z)?pt.z:z_min;
			}

		}
	}

	switch(rule)
	{
		case(FIT_BOUNDS):
		{
			std::cout << "calculate meter bounds\n";
			x_min = - (map_height * resolution) / 2;
			x_max = - x_min;

			y_min = - (map_width * resolution) / 2;
			y_max = - y_min;
			break;
		}
		case(FIT_RES_DYNAMIC_BOUNDS):
		case(FIT_RES):
		{
			std::cout << "Calculate resolution\n";
			resolution = std::min((x_max - x_min) / map_height, (y_max - y_min) / map_width);
			break;
		}
		case(FIT_CELLS_UNIT_RES):
		case(FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES):
		{
			std::cout << "Unit resolution\n";
			resolution = DEFAULT_RES;
		}
		case(FIT_CELLS_DYNAMIC_BOUNDS):
		case(FIT_CELLS):
		{
			std::cout << "Calculate map cell bounds\n"
;			map_width = std::ceil((y_max - y_min) / resolution);
			map_height = std::ceil((x_max - x_min) / resolution);
			break;
		}
		case(HOLD_ALL):
		{
			break;
		}
	}

	if ((initialize && use_first) || (!use_first))
	{
		//std::cout << "Map height, width: " << map_height << ", " << map_width << "\n";
		grid = Eigen::MatrixXf::Zero(map_height, map_width);
		grid_bytes.resize(map_height * map_width);
		map_grid_bytes.resize(map_height * map_width);
	} else
	{
		grid = Eigen::MatrixXf::Zero(grid.rows(), grid.cols());
	}
	grid_bytes.clear();
	map_grid_bytes.clear();

	int skip_count = 0;
	int pass_count = 0;
	//fill grid --------------------------------------------------------
	for (pointstruct pt : cloud)
	{
		if (pt.z < z_min || pt.z > z_max ||
			pt.x < x_min || pt.x > x_max ||
			pt.y < y_min || pt.y > y_max)
		{
			skip_count++;
			continue;
		} else
		{
			pass_count++;
		}

		uint i = 0;
		uint j = 0;

		//find index of point
		//Note: cell index rounds down from partial indices
		//i = (-pt.x + ( (x_max - x_min) / 2)) / resolution;
		//j = (-pt.y + ( (y_max - y_min) / 2)) / resolution;

		//i = (1 - (pt.x - x_min) / (map_height * resolution)) * map_height;
		//j = (1 - (pt.y - y_min) / (map_width * resolution)) * map_width;

		if (centered_x && centered_y)
		{
			i = (-pt.x ) / resolution + ( map_height / 2);
			j = (-pt.y ) / resolution + ( map_width / 2);
		} else if (centered_x)
		{
			i = (-pt.x ) / resolution + ( map_height / 2);
			j = (1 - (pt.y - y_min) / (map_width * resolution)) * map_width;
		} else if (centered_y)
		{
			i = (1 - (pt.x - x_min) / (map_height * resolution)) * map_height;	
			j = (-pt.y ) / resolution + ( map_width / 2);
		} else
		{
			i = (1 - (pt.x - x_min) / (map_height * resolution)) * map_height;
			j = (1 - (pt.y - y_min) / (map_width * resolution)) * map_width;
		}

		// i = (map_height - (pt.x - x_min) / (resolution));
		// j = (map_width - (pt.y - y_min) / (resolution));

		//if point within bounds, use it to populate the grid
		if (i < map_height && j < map_width && i >= 0 && j >= 0)
		{
			//increase grid count

			if (make_binary_map)
			{
				map_grid_bytes[map_width * (map_height - i) + (map_width - j)] = 255;
				grid_bytes[map_width * i + j] = 255;
			} else
			{
				grid(i,j) = std::max(grid(i,j), pt.z); //real-valued height grid
				grid_bytes[map_width * i + j] = std::round(std::min( ( std::max( (grid(i,j) - z_scale_min), 0.0) * 256.0 ) / (z_scale_max - z_scale_min), 256.0)); //scaled value height grid
				//value = std::min( ( std::max( (grid(i,j) - z_scale_min), 0.0) * 256.0 ) / (z_scale_max - z_scale_min), 256.0); 
				//std::cout << "adjusted: " << grid(i,j) - z_scale_min << " range: " << z_scale_max - z_scale_min << std::endl;
				//std::cout << "Target value: " << std::min( ( std::max( (grid(i,j) - z_scale_min), 0.0) * 256.0 ) / (z_scale_max - z_scale_min), 256.0) << std::endl;
				//std::cout << "Z value: " << grid(i,j) << " converted: " << (float) grid_bytes[map_width * i + j] << "\n";
				if (j < map_height && i < map_width)
				{
					//TODO: this should be the same as the grid_bytes, yeah?
					map_grid_bytes[map_width * (map_height - i) + (map_width - j)] = (grid(i,j) * 256) / (z_scale_max - z_scale_min); //scaled value, flipped height grid
				}
			}
		} 		
	}
	std::cout << "Out-of-bounds point count: " << skip_count << "\n In-bounds point count: " << pass_count << "\n";
}

void reconfigureCallback(pointcloud_utils::PointCloudUtilsConfig &config, uint32_t level)
{
	// new_resolution = config.map_resolution;
	// new_z_scale_min = config.z_scale_min;
	// new_z_scale_max = config.z_scale_max;
	// new_map_height =  config.map_height;
	// new_map_width = config.map_width;
	// has_new_params = true;

	resolution = config.map_resolution;
	z_scale_min = config.z_scale_min;
	z_scale_max = config.z_scale_max;
	centered_x = config.x_centered;
	centered_y = config.y_centered;

	x_min = config.x_min;
	x_max = config.x_max;
	y_min = config.y_min;
	y_max = config.y_max;
	z_min = config.z_min;
	z_max = config.z_max;

	has_new_params = true;
}

/**
 * @Function 	pointCloudCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	static bool first = true;

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

	header = msg->header;
	sensor_msgs::PointCloud2 base_cloud;
	transformToOutputFrame(msg, base_cloud);
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointstruct> cloud;
	cloud.resize(msg->width);
	std::memcpy(&(cloud[0]), &(base_cloud.data[0]), msg->row_step);


	//parse into grid
	parseGrid(cloud, grid, first);

	//std::cout << "grid: \n" << grid << "\n\n"; 

	publishMap();
	publishImage();

	if (first)
	{
		first = false;
	}
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

	//get params
	std::string lidar_topic;
	std::string image_pub_topic;
	std::string costmap_pub_topic;
	n_.param<std::string>("cloud_topic", lidar_topic, "/cloud");
	n_.param<std::string>("base_frame", base_frame, "base_link");
	n_.param<std::string>("lidar_frame", lidar_frame, "cloud");
	n_.param<std::string>("image_pub_topic", image_pub_topic, "/grid_image");
	n_.param<std::string>("costmap_pub_topic", costmap_pub_topic, "/map");
	n_.param<double>("map_resolution", resolution, 0.5);
	n_.param<int>("map_width", map_width, 512);
	n_.param<int>("map_height", map_height, 512);

	n_.param<bool>("centered_x", centered_x, false);
	n_.param<bool>("centered_y", centered_y, true);
   
    n_.param<bool>("constant_map_size", const_size, false);
    n_.param<bool>("constant_map_resolution", const_res, false);
    n_.param<bool>("use_bounds", use_bounds, false);
    n_.param<bool>("recalculate_dynamic_bounds", use_first, false);
   
    n_.param<double>("x_min", x_min, -10);
    n_.param<double>("x_max", x_max, 10);
    n_.param<double>("y_min", y_min, -10);
    n_.param<double>("y_max", y_max, 10);
    n_.param<double>("z_min", z_min, -10);
    n_.param<double>("z_max", z_max, 10);

    n_.param<double>("z_scale_max", z_scale_max, 10);
    n_.param<double>("z_scale_min", z_scale_min, 10);
    n_.param<bool>("binary_map", make_binary_map, false);

    ros::Subscriber tf_sub = n.subscribe("/tf", 1, &tfCallback);

    while (ros::ok() && !transform_found)
    {
    	ros::Rate(10).sleep();
    	std::cout << "waiting for transform.\n";
    	ros::spinOnce();
    }

    tf_sub.shutdown();

    dynamic_reconfigure::Server<pointcloud_utils::PointCloudUtilsConfig> server;
  	dynamic_reconfigure::Server<pointcloud_utils::PointCloudUtilsConfig>::CallbackType f;

    f = boost::bind(&reconfigureCallback, _1, _2);
  	server.setCallback(f);

	//subscribe/advertise
	map_pub = n.advertise<nav_msgs::OccupancyGrid>(costmap_pub_topic, 1);
	image_pub = n.advertise<sensor_msgs::Image>(image_pub_topic, 1);
	ros::Subscriber cloud_sub = n.subscribe(lidar_topic, 1, &pointCloudCallback);

	ros::spin();
	return(0);
}