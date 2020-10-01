/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Aug 2020
 * Brief: simple common utilities for pointlcoud operations
 * File: pointcloud_utils_impl.hpp
 */

#ifndef POINTCLOUD_UTILS_IMPL_HPP
#define POINTCLOUD_UTILS_IMPL_HPP

// -------------------------------
#include "pointcloud_utils/pointcloud_utils.hpp"
// -------------------------------

namespace pointcloud_utils
{
	/**
	 * @function getIntensity
	 * @brief    retrieves the intensity of the given point, if it exists
	 * @param    data - point that may have an intensity value
	 * @return 	 float* - pointer to intensity value, if it exists for the given point. Otherwise, will return NULL
	 */
	template <class T> inline const float* getIntensity(const T& data) { return NULL; }  
	template<> inline const float* getIntensity<pointstruct>(const pointcloud_utils::pointstruct& data) { return &data.intensity; }

	/**
	 * @function inTolerance
	 * @brief    checks if the two given data are within tolerance of each other
	 * @param    data1 		- first data to compare
	 * @param 	 data2 		- second data to compare
	 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
	 * @return 	 bool - true if data1 and data2 are within tolerance of each other
	 * 				  - false otherwise 
	 */
	inline bool inTolerance(const float data1, const float data2, const float tolerance) //TODO: this could be an in-line function
	{
		if ( fabs(data1 - data2) <= tolerance )
		{
			return true;
		}
	
		return false;
	}

	// /** 
	//  * @function 	convertToPointCloud2
	//  * @brief 		convert the given point vector into a pointcloud 2 message
	//  * @param 		cloud - pointstruct vector to convert
	//  * @param 		msg - pointcloud2 message to save conversion into
	//  * @return 		void
	//  */
	// template <class T> void convertToPointCloud2(const std::vector<T>& cloud, sensor_msgs::PointCloud2& msg)
	// {
	// 	//Note! Nothing to do in the default class :/
	// 	std::cout << "Unknown point type. Cannot complete conversion.\n";
	// }
	// template<> void convertToPointCloud2<pointsruct>(const std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::PointCloud2& msg)
	// {
	// 	msg.fields = pointcloud_utils::pointstruct_fields;
	// 	msg.point_step = pointcloud_utils::pointstruct_pointstep;
	// 	msg.width = cloud.size();
	// 	msg.height = 1;
	// 	msg.row_step = msg.pointstep * msg.width;
	// 	msg.data.resize(msg.row_step);
	// 	memcpy(&(msg.data[0]), &(cloud[0]), msg.row_step);
	// 
	// }

	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud, std::vector<T>& point_vector)
	{
		//memcpy into a struct (parses the data into the struct values)
		//here, we assume the user knows what they are doing, and print out a statement if it fails
		try
		{
			point_vector.resize(cloud->width);
			memcpy(&(point_vector[0]), &(cloud->data[0]), cloud->row_step);
		} catch (...)
		{
			std::cout << "Error: Cannot complete conversion. Is the poinstruct type correct for your cloud?\n";
		}
	}
	//template<> void convertFromPointCloud2<pointsruct>(const sensor_msgs::PointCloud2::ConstPtr& cloud, std::vector<pointcloud_utils::pointstruct>& point_vector)
	//{
	//	point_vector.resize(cloud->width);
	//	memcpy(&(point_vector[0]), &(cloud->data[0]), cloud.row_step);
	//}

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_IMPL_HPP
