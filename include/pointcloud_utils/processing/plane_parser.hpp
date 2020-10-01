/** Author: Stephanie Meyer swmeyer16@gmail.com 30 September 2020
 * Brief: a class to convert a 3D pointcloud into a 2D laserscan message
 * File: pointcloud_to_laserscan.hpp
 */

#ifndef POINTCLOUD_TO_LASERSCAN_HPP
#define POINTCLOUD_TO_LASERSCAN_HPP

// --------------------------
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

namespace pointcloud_utils
{
	class PointCloudToLaserScanConverter
	{
	public:

		struct Settings
		{
			float target_vertical_angle; 	// [rad] vertical angle at which to build the laserscan
			float vertical_angle_tolerance; // [rad] tolerance to accept a point into the laser scan
	
			bool filter_by_height; 			// if true, will use the target height as a selecting factor, rather than vertical angle
			float target_height; 			// [m] height of laserscan origin in original frame 
			float height_tolerance; 		//  m] tolerance to accept a point into the laserscan
		
			std::string scan_frame; 		// name of the frame to assign to the converted laser scan message
	
			bool use_fixed_resolution; 		// if true, the scan will use a fixed rather than calculated resolution. This will usually generate more accurate but non-dense scans
			float resolution; 				// [rad] resolution to use if fixed resoltion is desired
		
			float min_angle;				// [rad] minimum angle from 0.0 (positive x axis) desired in scan
			float max_angle; 				// [rad] maximum angle from 0.0 (positive x axis) desired in scan
			
			bool limit_front_distance; 		// if true, ignore points after a certain x distance (for removal of ground points for a down-facing angle)
			float front_distance_limit; 	// [m] distance in x to limit included points to, if limit_front_distance is true
		};

		PointCloudToLaserScanConverter(const PointCloudToLaserScanConverter::Settings& settings);
		~PointCloudToLaserScanConverter();

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	convertCloud
		 * @brief 		converts the given point cloud message into a 2D laserscan message
		 * @param 		cloud_in - inputted 3D point cloud
		 * @param 		cloud - space to store converted 3D cloud
		 * @param 		scan  - space to store the converted laserscan
		 * @return 		void
		 */
		void convertCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::LaserScan& scan);
		
	private:
		// --------------------------
		Settings settings; // holds the current settings for this converter
		// --------------------------


	}; //end class PointCloudGToLaserScanConverter

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_TO_LASERSCAN_HPP 