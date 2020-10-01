/** Author: Stephanie Meyer swmeyer16@gmail.com 1 July 2020
 * Brief: simple common utilities for pointlcoud operations
 * File: pointcloud_utils.hpp
 */

#ifndef POINTCLOUD_UTILS_HPP
#define POINTCLOUD_UTILS_HPP

// -------------------------------
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
// -------------------------------

namespace pointcloud_utils
{

	const float PI = 3.1415;

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

	} pointstruct; //old forestry trucks file velodyne format

	//std::vector<sensor_msgs::PointField> pointstruct_fields =
	//{
	//	{ "x", 0, sensor_msgs::PointField::FLOAT32, 1},
	//	{ "y", 4, sensor_msgs::PointField::FLOAT32, 1},
	//	{ "z", 8, sensor_msgs::PointField::FLOAT32, 1},
	//	{ "intensity", 16, sensor_msgs::PointField::FLOAT32, 1}
	//};
//
	//int pointstruct_step_size = 32; //TODO: does this work for every one?

	typedef struct
	{
		float x;
		float y;
		float z;
	} simplePointstruct;

	enum costmapValues
	{
		FREE = 255,
		UNKNOWN = 127,
		OCCUPIED = 0
	};

	typedef struct
	{
		float azimuth;
		float range;
		float vertical_angle;
	} sphericalPointstruct;

	typedef struct
	{
		float azimuth;
		float radius;
		float z;
	} polarPointstruct;

	/**
	 * @function getIntensity
	 * @brief    retrieves the intensity of the given point, if it exists
	 * @param    data - point that may have an intensity value
	 * @return 	 float* - pointer to intensity value, if it exists for the given point. Otherwise, will return NULL
	 */
	template <class T> inline const float* getIntensity(const T& data); 
	template<> inline const float* getIntensity<pointstruct>(const pointcloud_utils::pointstruct& data);

	/**
	 * @function inTolerance
	 * @brief    checks if the two given data are within tolerance of each other
	 * @param    data1 		- first data to compare
	 * @param 	 data2 		- second data to compare
	 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
	 * @return 	 bool - true if data1 and data2 are within tolerance of each other
	 * 				  - false otherwise 
	 */
	inline bool inTolerance(const float data1, const float data2, const float tolerance);

	// //TODO: finish making conversion functions for to- and from- pointstruct vectors and pointcloud2 messages
	// /** 
	//  * @function 	convertToPointCloud2
	//  * @brief 		convert the given point vector into a pointcloud 2 message
	//  * @param 		cloud - pointstruct vector to convert
	//  * @param 		msg - pointcloud2 message to save conversion into
	//  * @return 		void
	//  */
	// template <class T> void convertToPointCloud2(const std::vector<T>& cloud, sensor_msgs::PointCloud2& msg);
	// template<> void convertToPointCloud2<pointsruct>(const std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::PointCloud2& msg);
	

	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud, std::vector<T>& point_vector);
	
} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_HPP