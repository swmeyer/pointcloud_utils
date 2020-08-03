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
} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_IMPL_HPP