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
	} shellPointstruct;

	enum costmapValues
	{
		FREE = 255,
		UNKNOWN = 127,
		OCCUPIED = 0
	};
} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_HPP