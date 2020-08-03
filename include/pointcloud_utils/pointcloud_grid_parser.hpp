/** Author: Stephanie Meyer swmeyer16@gmail.com 1 July 2020
 * Brief: a class to parse pointcloud messages into grid images
 * File: pointcloud_grid_parser.hpp
 */

#ifndef POINTCLOUD_GRID_PARSER_HPP
#define POINTCLOUD_GRID_PARSER_HPP

// -------------------------------
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
			double resolution;			// [m/cell] grid resolution
			int map_width;				//num cells in width
			int map_height; 			//num cells in height
			
			bool centered_x;
			bool centered_y;
			
			int new_map_width;
			int new_map_height;
			int new_resolution;
			int new_z_scale_min;
			int new_z_scale_max;

			bool const_res;				//flag to freeze map resolution
			bool const_size; 			//flag to limit map by cell num
			bool use_bounds; 			//flag to use bounds on area of interest or not
			bool use_first; 			//flag to control whether dynamic bounds are found once or every time
			bool use_raytrace_to_clear_space; //flag to control whether to mark a three-value costmap or otherwise (note: overrides binary map)
			bool make_binary_map; //if true, occupied cells have value 255, unoccupied 0	
			bool use_shell_pointstruct; //if true, use the shell-style pointstruct to parse the cloud

			// Area of interest bounds:
			double x_min;
			double x_max;
			double y_min;
			double y_max;
			double z_min;
			double z_max;

			//Intensity bounds:
			double min_intensity;
			double max_intensity;
			
			double z_scale_max;
			double z_scale_min; //scaling parameters for converting between data types
			} Settings;

		void setSettings(Settings& settings);
		void getSettings(Settings& settings);

		void setTransform(geometry_msgs::TransformStamped& transform);

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
		bool init(PointCloudGridParser::Settings& settings, geometry_msgs::TransformStamped& transform);

		/**
		 * @Function 	updateCloud
		 * @Param 		msg - incoming data message
		 * @Param 		grid_image - space to store the updated map in after the new cloud (msg) is parsed and combined with any existing data
	 	 * @Param 		map - space to store the updated map, in costmap (OccupancyGrid) data format
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages.
		 */
		template <class T> void updateCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<uint8_t>& grid_image, std::vector<uint8_t>& map);
	
	
	private:
		std_msgs::Header header; //Header from most recent pointcloud message

		std::vector<pointcloud_utils::simplePointstruct> last_cartesian_cloud;
		std::vector<pointcloud_utils::polarPointstruct>  last_polar_cloud;

		geometry_msgs::TransformStamped transform;

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
		    const sensor_msgs::PointCloud2::ConstPtr& input_msg,
		    sensor_msgs::PointCloud2& output_msg
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