/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Jun 2020
 * Brief: converts a 3D pointcloud into a 2D laserscan message
 * File: pointcloud_to_laserscan.cpp
 */

// --------------------------
#include <pointcloud_utils/conversion/pointcloud_to_laserscan.hpp>

#include <pointcloud_utils/pointcloud_utils_impl.hpp> //Note: The users of this don't need this include!
// --------------------------

namespace pointcloud_utils
{
	PointCloudToLaserScanConverter::PointCloudToLaserScanConverter(const PointCloudToLaserScanConverter::Settings& settings)
	{
		this->settings = settings;
	}

	PointCloudToLaserScanConverter::~PointCloudToLaserScanConverter()
	{

	}

	/**
	 * @function 	convertCloud
	 * @brief 		converts the given point cloud message into a 2D laserscan message
	 * @param 		cloud_in - inputted 3D point cloud
	 * @param 		cloud - space to store converted 3D cloud
	 * @param 		scan  - space to store the converted laserscan
	 * @return 		void
	 */
	void PointCloudToLaserScanConverter::convertCloud(const sensor_msgs::msg::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::msg::LaserScan& scan)
	{
		//convert cloud to 2D and republish!

		//Set up scan message
		scan.header = cloud_in->header;
		scan.header.frame_id = settings.scan_frame;
		scan.range_min = 0;
		scan.range_max = 500; //TODO: this is absurdly large so shouldn't be a problem for dropping points, but might need to be adjusted in some applications
		scan.angle_min = settings.min_angle;
		scan.angle_max = settings.max_angle;


		//convert pointcloud
		cloud.resize(cloud_in->width);
		std::memcpy(&(cloud[0]), &(cloud_in->data[0]), cloud_in->row_step);
	
		//parse points into laser scan
		std::vector<pointcloud_utils::sphericalPointstruct> ring;

		for (pointcloud_utils::pointstruct pt : cloud)
		{
			pointcloud_utils::sphericalPointstruct spherical_pt;

			//std::cout << "point\n";
			spherical_pt.range = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
			spherical_pt.vertical_angle = std::atan2(pt.z, spherical_pt.range);
			spherical_pt.azimuth = std::atan2(pt.y, pt.x);

			//Skip non-selected points:
			if (spherical_pt.azimuth > settings.max_angle || spherical_pt.azimuth < settings.min_angle)
			{
				///std::cout << "Skipping pt\n";
				continue; //point is not within the target horizonal field of view
			}
			if (settings.limit_front_distance)
			{
				if (pt.x > settings.front_distance_limit)
				{
					//std::cout << "Skipping pt\n";
					continue;
				}
			}

			if (settings.filter_by_height)
			{
				//std::cout << "filtering by height. Target: " << settings.target_height << " at tolerance " << settings.height_tolerance << "\n";
				//std::cout << "Height: " << pt.z << "\n";
				if (!pointcloud_utils::inTolerance(pt.z, settings.target_height, settings.height_tolerance))
				{
					//std::cout << "Skipping pt\n";
					continue; //point is not in the target height ring! pass
				}


			} else
			{
				//std::cout << "filtering by vertical angle. Target: " << settings.target_vertical_angle << " at tolerance " << settings.vertical_angle_tolerance << "\n";
				if (!pointcloud_utils::inTolerance(spherical_pt.vertical_angle, settings.target_vertical_angle, settings.vertical_angle_tolerance))
				{
					//std::cout << "Skipping pt\n";
					continue; //point is not in the target vertical angle ring! pass
				}
			}
			
			//std::cout << "Point in tolerance\n";
			ring.push_back(spherical_pt);
		}

		int num_pts;
		float resolution;

		if (settings.use_fixed_resolution)
		{
			resolution = settings.resolution;
			num_pts = (settings.max_angle - settings.min_angle) / resolution;
		} else
		{
			num_pts = ring.size();
			resolution = (settings.max_angle - settings.min_angle) / num_pts;
		}

		scan.ranges.resize(num_pts, std::numeric_limits<float>::infinity());
		scan.angle_increment = resolution;

		int increment_index;
		for (pointcloud_utils::sphericalPointstruct pt : ring)
		{
			//iterate through ring and populate scan
			increment_index = (pt.azimuth - settings.min_angle) / resolution;
			scan.ranges[increment_index] = pt.range;
		}
	}

} //end namespace pointlcoud_utils