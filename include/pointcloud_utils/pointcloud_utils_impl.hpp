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
	template  <class T> float* getIntensity(T& data) { return NULL; }  

	template<> float* getIntensity<pointstruct>(pointcloud_utils::pointstruct& data) { return &data.intensity; }

	/**
	 * @function inTolerance
	 * @brief    checks if the two given data are within tolerance of each other
	 * @param    data1 		- first data to compare
	 * @param 	 data2 		- second data to compare
	 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
	 * @return 	 bool - true if data1 and data2 are within tolerance of each other
	 * 				  - false otherwise 
	 */
	bool inTolerance(float data1, float data2, float tolerance) //TODO: this could be an in-line function
	{
		if ( fabs(data1 - data2) <= tolerance )
		{
			return true;
		}
	
		return false;
	}
} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_IMPL_HPP