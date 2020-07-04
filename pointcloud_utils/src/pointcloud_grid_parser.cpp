/** Author: Stephanie Meyer swmeyer16@gmail.com 17 Feb 2020
/* Brief: a class to subscribe to point cloud messages and parse them into 2D grids
/* File: pointcloud_grid_parser.cpp
*/

//TODO: remove grid?, remove has_new_params?

// --------------------------
#include "pointcloud_utils/pointcloud_grid_parser.hpp"
// --------------------------

// --------------------------
namespace pointcloud_utils
{
	PointCloudGridParser::PointCloudGridParser():
		initialized(false),
		has_new_params(false)
	{
		//empty constructor
	}

	PointCloudGridParser::~PointCloudGridParser()
	{
		//empty destructor
	}

	/**
	 * @function init
	 * @brief    initialize this class so it can function
	 * @param    settings - settings to initialize with
	 * @param    transform - transform to intialize with (between cloud frame and robot base frame)
	 * @return   bool     - true if class is ready to work
	 */
	bool PointCloudGridParser::init(PointCloudGridParser::Settings& settings, geometry_msgs::TransformStamped& transform)
	{
		setSettings(settings);
		setTransform(transform);
		this->initialized = true;
		return true;
	}

	void PointCloudGridParser::setSettings(PointCloudGridParser::Settings& settings)
	{
		this->settings = settings;
		this->has_new_params = true;
	}

	void PointCloudGridParser::getSettings(PointCloudGridParser::Settings& settings)
	{
		settings = this->settings;
	}

	void PointCloudGridParser::setTransform(geometry_msgs::TransformStamped& transform)
	{
		this->transform = transform;
	}


	void PointCloudGridParser::getMapBytes(std::vector<uint8_t>& bytes)
	{
		bytes = this->grid_bytes;
	}

	void PointCloudGridParser::getGridBytes(std::vector<uint8_t>& bytes)
	{
		bytes = this->map_grid_bytes;
	}


	void PointCloudGridParser::getLastCartesianCloud(std::vector<pointcloud_utils::simplePointstruct>& cloud)
	{
		cloud = this->last_cartesian_cloud;
	}

	void PointCloudGridParser::getLastPolarCloud(std::vector<pointcloud_utils::polarPointstruct>& cloud)
	{
		cloud = this->last_polar_cloud;
	}


	//========================================
	/// @fn         transformToOutputFrame
	/// @brief      handle transform to base frame
	/// @param      input_msg - untransformed message
	/// @param      output_msg - holder for all transformed messages
	/// @return     bool - true if the transform is successful, else false
	/// @details    Converts the given input to the common output frame
	/// @author     Stephanie Meyer
	//========================================
	bool PointCloudGridParser::transformToOutputFrame
	(
	    const sensor_msgs::PointCloud2::ConstPtr& input_msg,
	    sensor_msgs::PointCloud2& output_msg
	)
	{
		tf2::doTransform (*input_msg, output_msg, transform);
		header.frame_id = output_msg.header.frame_id;
		return true;
	}

	//TODO: updateParams() method

	//TODO: raytraceParse() method

} //end namespace pointcloud_utils