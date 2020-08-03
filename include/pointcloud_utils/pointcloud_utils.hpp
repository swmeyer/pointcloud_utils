/** Author: Stephanie Meyer swmeyer16@gmail.com 1 July 2020
 * Brief: simple common utilities for pointlcoud operations
 * File: pointcloud_utils.hpp
 */

#ifndef POINTCLOUD_UTILS_HPP
#define POINTCLOUD_UTILS_HPP

// -------------------------------

// -------------------------------

namespace pointcloud_utils
{
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
		float z;
	} polarPointstruct;

	template  <class T> float* getIntensity(T& data); 

	template<> float* getIntensity<pointstruct>(pointcloud_utils::pointstruct& data);
} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_HPP