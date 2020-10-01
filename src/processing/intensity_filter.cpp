/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 8 Sept 2020
 * Brief: defines a class that republishes the incoming point cloud as a filtered cloud containing only the points with intensity
 *        within a given threshold range
 * File: intensity_filter.cpp
 */

// --------------------------
#include <pointcloud_utils/processing/intensity_filter.hpp>
#include <pointcloud_utils/pointcloud_utils_impl.hpp> //the users don't need this!
// --------------------------

namespace pointcloud_utils
{
	IntensityFilter::IntensityFilter()
	{

	}

	IntensityFilter::~IntensityFilter()
	{

	}

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
	void IntensityFilter::filterIntensity
	(
		const sensor_msgs::PointCloud2::ConstPtr& cloud_in, 
		std::vector<pointcloud_utils::pointstruct>& cloud, 
		sensor_msgs::PointCloud2& filtered_cloud,
		int intensity_min, 
		int intensity_max)
	{	
		// convert to point vector
		//cloud.resize(cloud_in->width);
		//std::memcpy(&(cloud[0]), &(cloud_in->data[0]), cloud_in->row_step);
		pointcloud_utils::convertFromPointCloud2(cloud_in, cloud); //TODO: test

		std::vector<pointcloud_utils::pointstruct> cloud_parsed;

		//for each point in the cloud, push it to the cloud_parsed if the intensity is within range
		for (pointcloud_utils::pointstruct pt : cloud)
		{
			if (pointcloud_utils::getIntensity(pt))
			{
				if (*pointcloud_utils::getIntensity(pt) <= intensity_max && *pointcloud_utils::getIntensity(pt) >= intensity_min)
				cloud_parsed.push_back(pt);
			}
		}

		//convert back to pointcloud message:
		filtered_cloud.header = cloud_in->header;
		filtered_cloud.fields = cloud_in->fields;
		filtered_cloud.point_step = cloud_in->point_step;
		filtered_cloud.height = 1;
		filtered_cloud.width = cloud_parsed.size();
		filtered_cloud.row_step = filtered_cloud.point_step * cloud_parsed.size();
		
		filtered_cloud.data.resize(filtered_cloud.row_step);
		memcpy(&(filtered_cloud.data[0]), &(cloud_parsed[0]), filtered_cloud.row_step);
	}

} //end namespace pointcloud_utils