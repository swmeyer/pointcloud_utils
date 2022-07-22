/** Author: Stephanie Meyer swmeyer16@gmail.com 1 July 2020
 * Brief: a class to parse pointcloud messages into grid images
 * File: pointcloud_grid_parser.hpp
 */

#ifndef POINTCLOUD_GRID_PARSER_HPP
#define POINTCLOUD_GRID_PARSER_HPP

// -------------------------------
// #include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/image_encodings.hpp>
#include <Eigen/Dense>

// #include <dynamic_reconfigure/server.h>
// #include <pointcloud_utils/PointCloudUtilsConfig.h>

// #include <tf2_sensor_msgs/msg/tf2_sensor_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "pointcloud_utils/pointcloud_utils.hpp"
// -------------------------------

namespace pointcloud_utils
{
	class PointCloudGridParser
	{
	public:
		PointCloudGridParser();
		~PointCloudGridParser();
		
		typedef enum
		{
			FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES = 0,
			FIT_RES_DYNAMIC_BOUNDS,
			FIT_CELLS_DYNAMIC_BOUNDS,
			FIT_BOUNDS,
			FIT_CELLS_UNIT_RES,
			FIT_RES,
			FIT_CELLS,
			HOLD_ALL = 7
		} mapRules;

		typedef struct
		{
			bool use_luminar_pointstruct; //if true, use the luminar-style pointstruct to parse the cloud

			// Map Size settings
			double resolution;			// [m/cell] grid resolution
			int map_width;				//num cells in width
			int map_height; 			//num cells in height
			
			bool centered_x;
			bool centered_y;

			bool const_res;				//flag to freeze map resolution
			bool const_size; 			//flag to limit map by cell num
			bool use_bounds; 			//flag to use bounds on area of interest or not
			bool use_first; 			//flag to control whether dynamic bounds are found once or every time
			
			//Map size bounds
			double map_x_min;
			double map_x_max;
			double map_y_min;
			double map_y_max;
			double map_z_min;
			double map_z_max;


			//Map value settings

			bool use_raytrace_to_clear_space; //flag to control whether to mark a three-value costmap or otherwise (note: overrides binary map)
			bool make_binary_map; //if true, occupied cells have value 255, unoccupied 0
			bool make_intensity_map; //if true, make a cell value from intensity rather than from height
			bool make_height_map; //if true, use height as cell value

			double binary_threshold; //If binary map and intensity map and/or height map at once, this is the value threshold that must be reached to indicate a 1 value

			double height_scale;	// value to scale height by to make cell values
			double intensity_scale; // value to scale intensity by to make cell values

			
			//Filter settings

			// Area filter bounds:
			double x_min;
			double x_max;
			double y_min;
			double y_max;
			double z_min;
			double z_max;

			//Intensity filter bounds:
			double min_intensity;
			double max_intensity;

			int point_skip_num;
			
		} Settings;

		void setSettings(Settings& settings);
		void getSettings(Settings& settings);

		void setTransform(geometry_msgs::msg::TransformStamped& transform);

		void getMapBytes(std::vector<uint8_t>& bytes);
		void getGridBytes(std::vector<uint8_t>& bytes);

		void getLastCartesianCloud(std::vector<pointcloud_utils::simplePointstruct>& cloud);
		void getLastPolarCloud(std::vector<pointcloud_utils::polarPointstruct>& cloud);

		/**
		 * @function init
		 * @brief    initialize this class so it can function
		 * @param    settings - settings to initialize with
	 	 * @param    transform - transform to intialize with (between cloud frame and robot base frame)
		 * @return   bool     - true if class is ready to work
		 */
		bool init(PointCloudGridParser::Settings& settings, geometry_msgs::msg::TransformStamped& transform);

		/**
		 * @Function 	updateCloud
		 * @Param 		msg - incoming data message
		 * @Param 		grid_image - space to store the updated map in after the new cloud (msg) is parsed and combined with any existing data
	 	 * @Param 		map - space to store the updated map, in costmap (OccupancyGrid) data format
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages.
		 */
		template <class T> void updateCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, std::vector<uint8_t>& grid_image, std::vector<uint8_t>& map);
	
	
	private:
		std_msgs::msg::Header header; //Header from most recent pointcloud message

		std::vector<pointcloud_utils::simplePointstruct> last_cartesian_cloud;
		std::vector<pointcloud_utils::polarPointstruct>  last_polar_cloud;

		geometry_msgs::msg::TransformStamped transform;

		Eigen::MatrixXf grid;
		std::vector<uint8_t> grid_bytes;
		std::vector<uint8_t> map_grid_bytes;
		
		bool has_new_params;

		Settings settings;
		bool initialized; //whether this class is ready to function or not

		const double DEFAULT_RES = 1; //[m/cell] if const_res is false and conditions are undefined, use 1-1 res

		
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
		bool transformToOutputFrame
		(
		    const sensor_msgs::msg::PointCloud2::SharedPtr& input_msg,
		    sensor_msgs::msg::PointCloud2& output_msg
		);

		/**
		 * @function    determineMapParams
		 * @Brief       based on the current mapping rules, re-configure the current map params
		 * @param       cloud - current pointcloud to fit map to
		 * @Param 		initialize - flag to indicate to re-initialize the grid size or not
		 * @return      void
		 */
		template <class T> void determineMapParams(std::vector<T>& cloud, const bool initialize);
	

		/**
		 * @Function 	parseGrid
		 * @Param 		msg - incoming data message
		 * @Param 		grid - grid to parse into
		 * @Param 		initialize - flag to indicate to re-initialize the grid size or not
		 * @Return 		void
		 * @Brief 		Sorts the given point cloud into a 2D grid
		 */
		template <class T> void parseGrid(std::vector<T>& cloud, Eigen::MatrixXf& grid, const bool initialize);


		/** 
		 * @function   	parseGridRayTrace
		 * @brief      	convert the given cloud into a 2D grid image, using raytrace logic to fill out
		 *             	   known empty space between objects and the sensor origin
		 * @Param 		grid - grid to parse into
		 * @param      	initialize - flag to indicate to re-initialize the grid size or not
		 * @return 	   	void
		 */
		template <class T> void parseGridRayTrace(std::vector<T>& cloud, Eigen::MatrixXf& grid, const bool initialize);
	}; //end class PointCloudGridParser

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_GRID_PARSER_HPP