/** Author: Stephanie Meyer swmeyer16@gmail.com 3 July 2020
 * Brief: contains implementation of template methods for pointcloud_grid_parser.hpp
 * File: pointcloud_grid_parser_impl.hpp
 */

#ifndef POINTCLOUD_GRID_PARSER_IMPL_HPP
#define POINTCLOUD_GRID_PARSER_IMPL_HPP

// -------------------------------
#include "pointcloud_utils/pointcloud_grid_parser.hpp"
// -------------------------------

namespace pointcloud_utils
{
	/**
	 * @Function 	updateCloud
	 * @Param 		msg - incoming data message
	 * @Param 		grid_image - space to store the updated map in after the new cloud (msg) is parsed and combined with any existing data
	 * @Param 		map - space to store the updated map, in costmap (OccupancyGrid) data format
	 * @Return 		void
	 * @Brief 		Reacts to incoming pointcloud messages.
	 */
	template <class T> void PointCloudGridParser::updateCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<uint8_t>& grid_image, std::vector<uint8_t>& map)
	{
		static bool first = true;
		std::cout << "Updating cloud inside grid parser\n";
		header = msg->header;
		sensor_msgs::PointCloud2 base_cloud;
		transformToOutputFrame(msg, base_cloud);
		std::cout << "Cloud transformed\n";
		//memcpy into a struct (parses the data into the struct values)
		std::vector<T> cloud;
		cloud.resize(msg->width);
		std::memcpy(&(cloud[0]), &(base_cloud.data[0]), msg->row_step);
		std::cout << "Cloud parsed out to pointstruct\n";
	
		//parse into grid
		if (settings.use_raytrace_to_clear_space)
		{
			std::cout << "doing raytrace\n";
			parseGridRayTrace<T>(cloud, grid, first);
		} else
		{
			parseGrid<T>(cloud, grid, first);
		}
		std::cout << "Finsihed parsing into grid\n";
	
		//std::cout << "grid: \n" << grid << "\n\n"; 
	
		if (first)
		{
			first = false;
		}

		grid_image = this->grid_bytes;
		map = this->map_grid_bytes;
	}

	/**
	 * @function    determineMapParams
	 * @Brief       based on the current mapping rules, re-configure the current map params
	 * @param       cloud - current pointcloud to fit map to
	 * @Param 		initialize - flag to indicate to re-initialize the grid size or not
	 * @return      void
	 */
	template <class T> void PointCloudGridParser::determineMapParams(std::vector<T>& cloud, const bool initialize)
	{
		//Determine map params --------------------------------------------------
		unsigned char rule = 0;
		rule |= settings.const_res;
		rule = rule << 1;
		rule |= settings.const_size;
		rule = rule << 1;
		rule |= settings.use_bounds;
	
		if ((initialize && settings.use_first) || (!settings.use_first))
		{ //TODO: this num cells and resolution and bounds could be better handled by a quadtree
	
			//We'll have to go through the cloud twice, this first time to get grid size params
			for (T pt : cloud)
			{
				if (rule == mapRules::FIT_CELLS_DYNAMIC_BOUNDS || 
					 rule == mapRules::FIT_RES_DYNAMIC_BOUNDS ||
					 rule == mapRules::FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES)
				{
					settings.x_min = (settings.x_min > pt.x)?pt.x:settings.x_min;
					settings.y_min = (settings.y_min > pt.y)?pt.y:settings.y_min;

					settings.x_max = (settings.x_max < pt.x)?pt.x:settings.x_max;
					settings.y_max = (settings.y_max < pt.y)?pt.y:settings.y_max;

					settings.z_max = (settings.z_max < pt.z)?pt.z:settings.z_max;
					settings.z_min = (settings.z_min > pt.z)?pt.z:settings.z_min;
				}
	
			}
		}
	
		switch(rule)
		{
			case(FIT_BOUNDS):
			{
				std::cout << "calculate meter bounds\n";
				settings.x_min = - (settings.map_height * settings.resolution) / 2;
				settings.x_max = - settings.x_min;
	
				settings.y_min = - (settings.map_width * settings.resolution) / 2;
				settings.y_max = - settings.y_min;
				break;
			}
			case(FIT_RES_DYNAMIC_BOUNDS):
			case(FIT_RES):
			{
				std::cout << "Calculate resolution\n";
				settings.resolution = std::min((settings.x_max - settings.x_min) / settings.map_height, (settings.y_max - settings.y_min) / settings.map_width);
				break;
			}
			case(FIT_CELLS_UNIT_RES):
			case(FIT_CELLS_DYNAMIC_BOUNDS_UNIT_RES):
			{
				std::cout << "Unit resolution\n";
				settings.resolution = DEFAULT_RES;
			}
			case(FIT_CELLS_DYNAMIC_BOUNDS):
			case(FIT_CELLS):
			{
				std::cout << "Calculate map cell bounds\n";
				settings.map_width = std::ceil((settings.y_max - settings.y_min) / settings.resolution);
				settings.map_height = std::ceil((settings.x_max - settings.x_min) / settings.resolution);
				break;
			}
			case(HOLD_ALL):
			{
				break;
			}
		}
	}
	
	/**
	 * @Function 	parseGrid
	 * @Param 		cloud - incoming data message
	 * @Param 		grid - grid to parse into
	 * @Param 		initialize - flag to indicate to re-initialize the grid size or not
	 * @Return 		void
	 * @Brief 		Sorts the given point cloud into a 2D grid
	 */
	template <class T> void PointCloudGridParser::parseGrid(std::vector<T>& cloud, Eigen::MatrixXf& grid, const bool initialize)
	{
		std::cout << "Starting grid parsing\n";
		determineMapParams<T>(cloud, initialize);

		//reset map -------------------------------------------------
		if ((initialize && settings.use_first) || (!settings.use_first))
		{
			//std::cout << "Map height, width: " << map_height << ", " << map_width << "\n";
			grid = Eigen::MatrixXf::Zero(settings.map_height, settings.map_width);
			//grid_bytes.resize(settings.map_height * settings.map_width);
			//map_grid_bytes.resize(settings.map_height * settings.map_width);
			grid_bytes = std::vector<uint8_t>(settings.map_height * settings.map_width, 0);
			map_grid_bytes = std::vector<uint8_t>(settings.map_height * settings.map_width, 0);
		} else
		{
			grid = Eigen::MatrixXf::Zero(grid.rows(), grid.cols());
			grid_bytes = std::vector<uint8_t>(grid_bytes.size(), 0);
			map_grid_bytes = std::vector<uint8_t>(map_grid_bytes.size(), 0);
		}
	
		int skip_count = 0;
		int pass_count = 0;
		//fill grid --------------------------------------------------------
		std::cout << "Z min: " << settings.z_min << "\n";
		for (T pt : cloud)
		{
			if (pt.z < settings.z_min || pt.z > settings.z_max ||
				pt.x < settings.x_min || pt.x > settings.x_max ||
				pt.y < settings.y_min || pt.y > settings.y_max)
			{
				skip_count++;
				continue;
			} else
			{
				pass_count++;
			}
	
			uint i = 0;
			uint j = 0;
	
			//find index of point
			//Note: cell index rounds down from partial indices
			//i = (-pt.x + ( (x_max - x_min) / 2)) / resolution;
			//j = (-pt.y + ( (y_max - y_min) / 2)) / resolution;
	
			//i = (1 - (pt.x - x_min) / (map_height * resolution)) * map_height;
			//j = (1 - (pt.y - y_min) / (map_width * resolution)) * map_width;
	
			if (settings.centered_x && settings.centered_y)
			{
				//TODO: should these be negative pt.x and pt.y, or not??
				i = (pt.x ) / settings.resolution + ( settings.map_height / 2);
				j = (pt.y ) / settings.resolution + ( settings.map_width / 2);
				//std::cout << "Resolution: " << settings.resolution << ", height: " << settings.map_height << ", " << settings.map_width << "\n";
			} else if (settings.centered_x)
			{
				i = (pt.x ) / settings.resolution + ( settings.map_height / 2);
				j = ((pt.y - settings.y_min) / (settings.map_width * settings.resolution)) * settings.map_width;
			} else if (settings.centered_y)
			{
				i = ((pt.x - settings.x_min) / (settings.map_height * settings.resolution)) * settings.map_height;	
				j = (pt.y ) / settings.resolution + ( settings.map_width / 2);
			} else
			{
				i = ((pt.x - settings.x_min) / (settings.map_height * settings.resolution)) * settings.map_height;
				j = ((pt.y - settings.y_min) / (settings.map_width * settings.resolution)) * settings.map_width;
			}
	
			// i = (map_height - (pt.x - x_min) / (resolution));
			// j = (map_width - (pt.y - y_min) / (resolution));
	
			//if point within bounds, use it to populate the grid
			if (i < settings.map_height && j < settings.map_width && i >= 0 && j >= 0)
			{
				//increase grid count
	
				if (settings.make_binary_map)
				{
					map_grid_bytes[settings.map_width * (settings.map_height - i) + (settings.map_width - j)] = 255;
					grid_bytes[settings.map_width * i + j] = 255;
				} else
				{
					grid(i,j) = std::max(grid(i,j), pt.z); //real-valued height grid
					grid_bytes[settings.map_width * i + j] = std::round(std::min( ( std::max( (grid(i,j) - settings.z_scale_min), 0.0) * 256.0 ) / (settings.z_scale_max - settings.z_scale_min), 256.0)); //scaled value height grid
					//value = std::min( ( std::max( (grid(i,j) - z_scale_min), 0.0) * 256.0 ) / (z_scale_max - z_scale_min), 256.0); 
					//std::cout << "adjusted: " << grid(i,j) - z_scale_min << " range: " << z_scale_max - z_scale_min << std::endl;
					//std::cout << "Target value: " << std::min( ( std::max( (grid(i,j) - z_scale_min), 0.0) * 256.0 ) / (z_scale_max - z_scale_min), 256.0) << std::endl;
					//std::cout << "Z value: " << grid(i,j) << " converted: " << (float) grid_bytes[map_width * i + j] << "\n";
					if (j < settings.map_height && i < settings.map_width)
					{
						//TODO: this should be the same as the grid_bytes, yeah?
						map_grid_bytes[settings.map_width * (settings.map_height - i) + (settings.map_width - j)] = (grid(i,j) * 256) / (settings.z_scale_max - settings.z_scale_min); //scaled value, flipped height grid
					}
				}
			} 		
		}
		std::cout << "Out-of-bounds point count: " << skip_count << "\n In-bounds point count: " << pass_count << "\n";
	}

	/** 
	 * @function   	parseGridRayTrace
	 * @brief      	convert the given cloud into a 2D grid image, using raytrace logic to fill out
	 *             	   known empty space between objects and the sensor origin
	 * @param      	grid - grid to parse into
	 * @param      	initialize - flag to indicate to re-initialize the grid size or not
	 * @return 	   	void
	 */
	template <class T> void PointCloudGridParser::parseGridRayTrace(std::vector<T>& cloud, Eigen::MatrixXf& grid, const bool initialize)
	{
		std::cout << "Starting grid parsing with ray trace\n";

		std::cout << "Z min: " << settings.z_min << "\n";

		determineMapParams<T>(cloud, initialize);

		//intialize grid -------------------------------------------------
		if ((initialize && settings.use_first) || (!settings.use_first))
		{
			//std::cout << "Map height, width: " << map_height << ", " << map_width << "\n";
			grid = Eigen::MatrixXf::Constant(settings.map_height, settings.map_width, pointcloud_utils::costmapValues::UNKNOWN);
			//grid_bytes.resize(settings.map_height * settings.map_width);
			//map_grid_bytes.resize(settings.map_height * settings.map_width);
			grid_bytes = std::vector<uint8_t>(settings.map_height * settings.map_width, pointcloud_utils::costmapValues::UNKNOWN);
			map_grid_bytes = std::vector<uint8_t>(settings.map_height * settings.map_width, pointcloud_utils::costmapValues::UNKNOWN);
		} else
		{
			grid = Eigen::MatrixXf::Constant(grid.rows(), grid.cols(), pointcloud_utils::costmapValues::UNKNOWN);
			grid_bytes = std::vector<uint8_t>(grid_bytes.size(), pointcloud_utils::costmapValues::UNKNOWN);
			map_grid_bytes = std::vector<uint8_t>(map_grid_bytes.size(), pointcloud_utils::costmapValues::UNKNOWN);
		}

		std::vector<pointcloud_utils::polarPointstruct> minimal_polar_cloud;
		std::vector<pointcloud_utils::simplePointstruct> minimal_cartesian_cloud;

		int skip_count = 0;
		int pass_count = 0;
		//fill grid -------------------------------------------------
		for (T pt : cloud) 
		{	
			//TODO: technically, we should place all the black points before ray-tracing...

			if (//pt.z < settings.z_min || pt.z > settings.z_max ||
				pt.x < settings.x_min || pt.x > settings.x_max ||
				pt.y < settings.y_min || pt.y > settings.y_max)
			{
				skip_count++;
				continue;
			} else
			{
				pass_count++;
			}
	
			uint i = 0;
			uint j = 0;

			uint map_center_i = 0;
			uint map_center_j = 0;

			//TODO: map from polar grid instead!!!!!
			if (settings.centered_x && settings.centered_y)
			{
				map_center_i = settings.map_height / 2;
				map_center_j = settings.map_width  / 2;

				i = (pt.x ) / settings.resolution + map_center_i;
				j = (pt.y ) / settings.resolution + map_center_j;
				
			} else if (settings.centered_x)
			{
				map_center_i = settings.map_height / 2;
				if (settings.y_min < 0)
				{
					map_center_j = ( fabs(settings.y_min) / settings.resolution);
				} else
				{
					map_center_j = settings.map_width;
				}

				i = (pt.x ) / settings.resolution + map_center_i;
				j = ((pt.y - settings.y_min) / (settings.map_width * settings.resolution)) * settings.map_width;
			} else if (settings.centered_y)
			{
				if (settings.x_min < 0)
				{
					//map_center_i = settings.map_height - ( fabs(settings.x_min) / settings.resolution);
					map_center_i = ( fabs(settings.x_min) / settings.resolution);
				} else
				{
					map_center_i = settings.map_height;
				}
				map_center_j = settings.map_width / 2;
				//i = (1 - (-pt.x - settings.x_min) / (settings.map_height * settings.resolution)) * settings.map_height;	
				i = ((pt.x - settings.x_min) / (settings.map_height * settings.resolution)) * settings.map_height;	
				j = (pt.y ) / settings.resolution + map_center_j;
			} else
			{
				if (settings.x_min < 0)
				{
					map_center_i = ( fabs(settings.x_min) / settings.resolution);
				} else
				{
					map_center_i = settings.map_height;
				}
				if (settings.y_min < 0)
				{
					map_center_j = ( fabs(settings.y_min) / settings.resolution);
				} else
				{
					map_center_j = settings.map_width;
				}
				i = ((pt.x - settings.x_min) / (settings.map_height * settings.resolution)) * settings.map_height;
				j = ((pt.y - settings.y_min) / (settings.map_width * settings.resolution)) * settings.map_width;
			}
	
			//if point within bounds, use it to populate the grid
			if (i < settings.map_height && j < settings.map_width && i >= 0 && j >= 0 && pt.z > settings.z_min && pt.z < settings.z_max)
			{
				//mark solid objects
				map_grid_bytes[settings.map_width * (settings.map_height - i) + (settings.map_width - j)] = pointcloud_utils::costmapValues::OCCUPIED;
				grid_bytes[settings.map_width * i + j] = pointcloud_utils::costmapValues::OCCUPIED;  
				pass_count++;
			} else
			{
				skip_count++;
			}	

			//ray-trace freespace
			//std::cout << "i, j , center i, center j: " << i << ", " << j << ", " << map_center_i << ", " << map_center_j << "\n";
			if (i == map_center_i)
			{
				//std::cout << "We found a horizontal line to trace!!!\n";

				int start_point, end_point;

				// always start from map center and fan outward
				end_point = std::min(std::max(j, (uint) 0), (uint) settings.map_width);
				start_point = std::min((int) map_center_j, end_point);
				end_point = std::max((int) map_center_j, end_point);

				// //Start at the lower of the detection point or map center (while still on the map), and end at the greater of the two (still on the map)
				// start_point = std::max(std::min(map_center_j, j), (uint) 0);
				// end_point = std::min(std::max(map_center_j, j), (uint) settings.map_width);
				
				int n = i;

				for (int m = start_point; m <= end_point; m++)
				{
					if (n < settings.map_height && m < settings.map_width && n >= 0 && m >= 0)
					{
						if (grid_bytes[settings.map_width * n + m] != pointcloud_utils::costmapValues::OCCUPIED)
						{
							//mark freespace
							map_grid_bytes[settings.map_width * (settings.map_height - n) + (settings.map_width - m)] = pointcloud_utils::costmapValues::FREE;
							grid_bytes[settings.map_width * n + m] = pointcloud_utils::costmapValues::FREE;  
						} else
						{
							//save minimal point
							pointcloud_utils::simplePointstruct min_cart;
							min_cart.x = n;
							min_cart.y = m;
							min_cart.z = 0; //todo: save the z from the real point
							minimal_cartesian_cloud.push_back(min_cart);

							pointcloud_utils::polarPointstruct min_polar;
							min_polar.azimuth = std::atan2(m, n); //rad
							min_polar.range = std::sqrt(std::pow(m, 2) + std::pow(n, 2));
							min_polar.z = min_cart.z;
							minimal_polar_cloud.push_back(min_polar);

							break;
						}
					}
				}
			} else
			{
				double x_diff = (double)i - (double)map_center_i;
				//std::cout << "x_diff: " << x_diff << "\n";
				double y_diff = (double)j - (double)map_center_j;
				//std::cout << "y_diff: " << y_diff << "\n";
				double a_slope = y_diff / x_diff;
				double b_intersection = j - (a_slope * i);
				//std::cout << "line equation: y = " << a_slope << " * x + " << b_intersection << "\n";
				
				int start_point, end_point;
				
				// always start from map center and fan outward
				end_point = std::min(std::max(i, (uint) 0), (uint) settings.map_height);
				start_point = std::min((int) map_center_i, end_point);
				end_point = std::max((int) map_center_i, end_point);

				// //Start at the lower of the detection point or map center (while still on the map), and end at the greater of the two (still on the map)
				// start_point = std::max(std::min(map_center_i, i), (uint) 0);
				// end_point = std::min(std::max(map_center_i, i), (uint) settings.map_height);
				
				//std::cout << "Starting loop for raytrace\n";
				//std::cout << "Start, end: " << start_point << ", " << end_point << "\n";
				for (int n = start_point; n <= end_point; n++)
				{
					int m = a_slope * n + b_intersection;
					//std::cout << "i, j: " << n << ", " << m << "\n";
					//if point within bounds, use it to populate the grid
					if (n < settings.map_height && m < settings.map_width && n >= 0 && m >= 0)
					{
						if (grid_bytes[settings.map_width * n + m] != pointcloud_utils::costmapValues::OCCUPIED)
						{
							//mark freespace
							map_grid_bytes[settings.map_width * (settings.map_height - n) + (settings.map_width - m)] = pointcloud_utils::costmapValues::FREE;
							grid_bytes[settings.map_width * n + m] = pointcloud_utils::costmapValues::FREE;  
						}  else
						{
							//save minimal point
							pointcloud_utils::simplePointstruct min_cart;
							min_cart.x = n;
							min_cart.y = m;
							min_cart.z = 0; //todo: save the z from the real point
							minimal_cartesian_cloud.push_back(min_cart);

							pointcloud_utils::polarPointstruct min_polar;
							min_polar.azimuth = std::atan2(m, n); //rad
							min_polar.range = std::sqrt(std::pow(m, 2) + std::pow(n, 2));
							min_polar.z = min_cart.z;
							minimal_polar_cloud.push_back(min_polar);

							break;
						}
					}
				}				
			}
		}

		last_cartesian_cloud = minimal_cartesian_cloud;
		last_polar_cloud = minimal_polar_cloud;
		std::cout << "Out-of-bounds point count: " << skip_count << "\n In-bounds point count: " << pass_count << "\n";
	}


} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_GRID_PARSER__IMPL_HPP