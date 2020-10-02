/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2020
 * Brief: defines a class that detects and prcesses ground. Functions include removal or 
 *        segmentation of ground points and alignment of scan to ground
 * File: ground_processor.hpp
 */

#ifndef GROUND_PROCESSOR_HPP
#define GROUND_PROCESSOR_HPP

// --------------------------
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <Eigen/Dense>

#include <pointcloud_utils/pointcloud_utils.hpp>
#include "pointcloud_utils/processing/plane_parser.hpp"
// --------------------------

namespace pointcloud_utils
{
	class GroundProcessor
	{
	public:

		struct Settings
		{
			PlaneParser::Settings plane_parser_settings; //setings for the plane parser instance
			PlaneParser::SearchWindow plane_search_window; //search window for the plane fit
			std::string aligned_cloud_frame; //name of frame to report ground-aligned scans in
			
			float point_to_plane_tolerance; //[m] maximum acceptable distance for a point to be away from the plane and still be considerd planar
		};

		GroundProcessor(GroundProcessor::Settings& settings);
		~GroundProcessor();

		void updateSettings(GroundProcessor::Settings& settings);

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	updateCloud
		 * @brief 		sets the current processing cloud to the given point cloud 
		 *				and identifies ground in the new cloud
		 * @param 		cloud_in - inputted 3D point cloud
		 * @param 		cloud - space to store converted 3D cloud
		 * @return 		void
		 */
		void updateCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud);
		
		/**
		 * @function 	returnPlaneDescriptors
		 * @brief 		sends back the detected plane coefficients
		 * @param 		plane_parameters - place to store the found plane parameters
		 * @param 		plane_states - place to store the found plane states
		 * @return 		void
		 */
		void returnPlaneDescriptors(PlaneParser::PlaneParameters& plane_parameters, PlaneParser::States& plane_states);

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	alignToGround
		 * @brief 		re-orients the given cloud so that the detected ground plane is level
		 * @param 		cloud_in - input cloud to align
		 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
		 * @param 		cloud - point vector version of the aligned cloud
		 * @return 		void
		 */
		void alignToGround(std::vector<pointcloud_utils::pointstruct>& cloud_in, sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud);
		
		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	alignToGround
		 * @brief 		re-orients the given cloud so that the detected ground plane is level
		 * @param 		cloud_in - input cloud to align
		 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
		 * @param 		cloud - point vector version of the aligned cloud
		 * @return 		void
		 */
		void alignToGround(sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud);

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	alignToGround
		 * @brief 		re-orients the given cloud so that the detected ground plane is level
		 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
		 * @param 		cloud - point vector version of the aligned cloud
		 * @return 		void
		 */
		void alignToGround(sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud);

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	separateGround
		 * @brief 		divides the current cloud into two sub-sets: one with the detected ground points, and one with everything else
		 * @param 		ground_msg - pointcloud2 version of the ground
		 * @param 		nonground_msg - pointcloud2 version of the non-ground points
		 * @param 		ground - point vector version of the ground
		 * @param 		nonground - point vector version of the non-ground points
		 * @return 		void
		 */
		void separateGround
		(
			sensor_msgs::PointCloud2& ground_msg, 
			sensor_msgs::PointCloud2& nonground_msg, 
			std::vector<pointcloud_utils::pointstruct>& ground,
			std::vector<pointcloud_utils::pointstruct>& nonground
		);
		
	private:
		// --------------------------
		PlaneParser* plane_parser;
		GroundProcessor::Settings settings;

		std::vector<pointcloud_utils::pointstruct> current_cloud;

		std_msgs::Header header;
		std::vector<sensor_msgs::PointField> fields;
		uint32_t point_step;

		//current ground stats:
		PlaneParser::PlaneParameters plane_parameters;
		PlaneParser::States plane_states;

		bool plane_detected;
		// --------------------------

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	transformCloud
		 * @brief 		transforms the given cloud by the given transform
		 * @param 		cloud - points to transform
		 * @param 		transform - transform to use
		 * @return 		void
		 */
		void transformCloud(std::vector<pointcloud_utils::pointstruct>& cloud, Eigen::Matrix4f& transform);

		//TODO: make a templated version which requires the specification of a pointstruct type
		/**
		 * @function 	detectGround
		 * @brief 		finds and parameterizes the ground plane from the given cloud
		 * @param 		cloud_in - pointcloud holding points to process
		 * @param 		cloud - place to store converted cloud
		 * @param 		plane_parameters - place to store detected ground plane parameters
		 * @param 		plane_states - place to store detected plane states
		 * @return 		void
		 */
		void detectGround(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud, PlaneParser::PlaneParameters& plane_parameters, PlaneParser::States& plane_states);
	}; //end class IntensityFilter

} //end namespace pointcloud_utils

#endif //end ifndef GROUND_RPCESSOR_HPP 