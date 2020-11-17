/** Author: Stephanie Meyer swmeyer16@gmail.com 12 Nov 2020
 * Brief: reads the point clouds from a given bagfile and publishes/provides the clouds upon request
 * File: pointcloud_from_bag.hpp
 */

//The plan is to make this a ros service

#ifndef POINTCLOUD_FROM_BAG_HPP
#define POINTCLOUD_FROM_BAG_HPP

// -------------------------------
#include <sensor_msgs/PointCloud2.h>
// -------------------------------

namespace pointcloud_utils
{
	class PointCloudFromBag
	{
		public:
			PointCloudFromBag();
			~PointCloudFromBag();
	};

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_FROM_BAG_HPP