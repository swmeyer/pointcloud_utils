/** Author: Stephanie Meyer swmeyer16@gmail.com 12 Nov 2020
 * Brief: adds the current time as the timestamp on successive LiDAR frames
 * File: pointcloud_stamper.hpp
 */

//Do I really need a class for this? maybe it is just a script

#ifndef POINTCLOUD_STAMPER_HPP
#define POINTCLOUD_STAMPER_HPP

// -------------------------------
#include <sensor_msgs/PointCloud2.h>
// -------------------------------

namespace pointcloud_utils
{
	class PointCloudStamper
	{
		public:
			PointCloudStamper();
			~PointCloudStamper();
	};

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_STAMPER_HPP