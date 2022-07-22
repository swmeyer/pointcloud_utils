/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 30 Sept 2020
 * Brief: defines a class that republishes the incoming point cloud as a filtered cloud containing only the points with intensity
 *        within a given threshold range
 * File: intensity_filter.hpp
 */

#ifndef INTENSITY_FILTER_HPP
#define INTENSITY_FILTER_HPP

// --------------------------
#include <sensor_msgs/msg/point_cloud_2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

namespace pointcloud_utils
{
	class IntensityFilter
	{
	public:

		IntensityFilter();
		~IntensityFilter();

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	filterIntensity
		 * @brief 		converts the given point cloud message into a 2D laserscan message
		 * @param 		cloud_in - inputted 3D point cloud
		 * @param 		cloud - space to store converted 3D cloud
		 * @param 		filtered_cloud  - space to store the converted laserscan
		 * @param 		intensity_min - minimum intensity value to include
		 * @param 		intensity_max - maximum intensity value to include
		 * @return 		void
		 */
		void filterIntensity
		(
			const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in, 
			std::vector<pointcloud_utils::pointstruct>& cloud, 
			sensor_msgs::msg::PointCloud2& filtered_cloud,
			int intensity_min, 
			int intensity_max);
		
	private:
		// --------------------------
		
		// --------------------------


	}; //end class IntensityFilter

} //end namespace pointcloud_utils

#endif //end ifndef INTENSITY_FILTER_HPP 