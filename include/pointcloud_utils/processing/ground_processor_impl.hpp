/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2020
 * Brief: defines a class that detects and prcesses ground. Functions include removal or 
 *        segmentation of ground points and alignment of scan to ground
 * File: ground_processor_impl.hpp
 */

#ifndef GROUND_PROCESSOR_IMPL_HPP
#define GROUND_PROCESSOR_IMPL_HPP

// --------------------------
#include <pointcloud_utils/processing/ground_processor.hpp>
#include <pointcloud_utils/pointcloud_utils_impl.hpp> //the users don't need this!
// --------------------------

//TODO: add an option to remove points only if the intensity is in tolerance of the average for the selected window
//TODO: for some reason, if alignment is called, it changes the results of the separation algorithm

namespace pointcloud_utils
{
	template <class T> GroundProcessor<T>::GroundProcessor(GroundProcessorSettings& settings)
	{
		plane_detected = false;
		this->settings = settings;
		this->plane_parser = new pointcloud_utils::PlaneParser<T>(settings.plane_parser_settings);
	}

	template <class T> GroundProcessor<T>::~GroundProcessor()
	{
		delete plane_parser;
	}

	template <class T> void GroundProcessor<T>::updateSettings(GroundProcessorSettings& settings)
	{
		this->settings = settings;
		plane_parser->updateSettings(settings.plane_parser_settings);
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	updateCloud
	 * @brief 		sets the current processing cloud to the given point cloud 
	 *				and identifies ground in the new cloud
	 * @param 		cloud_in - inputted 3D point cloud
	 * @param 		cloud - space to store converted 3D cloud
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::updateCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in, std::vector<T>& cloud)
	{
		//std::cout << "Points in: " << cloud_in->width << "\n";
		plane_detected = false;

		header = cloud_in->header;
		fields = cloud_in->fields;
		point_step = cloud_in->point_step;

		//detect ground
		plane_parser_utils::PlaneParameters plane_parameters;
		plane_parser_utils::States plane_states;
		//std::cout << "Detecting ground\n";
		detectGround(cloud_in, cloud, plane_parameters, plane_states);
		//std::cout << "Done detecting ground\n";
		this->current_cloud = cloud;
		this->plane_parameters = plane_parameters;
		this->plane_states = plane_states;
		// std::cout << "Plane coefficients: " << this->plane_parameters.a_d << ", " << this->plane_parameters.b_d << ", " << this->plane_parameters.c_d << "\n";
		
		// std::cout << "States roll and pitch: " << this->plane_states.roll << ", " << this->plane_states.pitch << "\n";

		plane_detected = true;
	}

	/**
	 * @function 	returnPlaneDescriptors
	 * @brief 		sends back the detected plane coefficients
	 * @param 		plane_parameters - place to store the found plane parameters
	 * @param 		plane_states - place to store the found plane states
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::returnPlaneDescriptors(plane_parser_utils::PlaneParameters& plane_parameters, plane_parser_utils::States& plane_states)
	{
		if (!plane_detected)
		{
			std::cout << "Warning! Ground has not been detected yet. no parameters to return.\n";
			return;
		}

		plane_parameters = this->plane_parameters;
		plane_states = this->plane_states;
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	alignToGround
	 * @brief 		re-orients the given cloud so that the detected ground plane is level
	 * @param 		cloud_in - input cloud to align
	 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
	 * @param 		cloud - point vector version of the aligned cloud
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::alignToGround(std::vector<T>& cloud_in, sensor_msgs::msg::PointCloud2& aligned_cloud, std::vector<T>& cloud)
	{
		if (!plane_detected)
		{
			std::cout << "Warning! Ground has not been detected yet. Cannot align to ground.\n";
			return;
		}

		//Save local copies
		plane_parser_utils::States local_plane_states = this->plane_states;

		// std::cout << "Roll, pitch: " << local_plane_states.roll << ", " << local_plane_states.pitch << "\n";
		float roll = local_plane_states.roll;
		float pitch = local_plane_states.pitch;

		//std::cout << "Roll and pitch rad: " << roll << ", " << pitch << "\n";
		//std::cout << "Translation: " << plane_states.x << ", " << plane_states.y << ", " << plane_states.z << "\n";

		//generate an affine transform in an eigen matrix from the current plane parameters
		Eigen::Matrix3f rotation_matrix;
		rotation_matrix << std::cos(pitch), 					0, 					std::sin(pitch),
						   std::sin(roll) * std::sin(pitch), 	std::cos(roll), 	-std::sin(roll) * std::cos(pitch),
						   -std::cos(roll) * std::sin(pitch), 	std::sin(roll), 	std::cos(roll) * std::cos(pitch);

		//NOTE: Since this is a ground plane, we neglect any yaw mesasure, as it is practically unobservable


		Eigen::Matrix4f transform;
		transform << rotation_matrix.row(0), 0,
					 rotation_matrix.row(1), 0,
					 rotation_matrix.row(2), - local_plane_states.z,
					 0, 0, 0, 				 1;


		transformCloud(cloud_in, transform);

		cloud = cloud_in;

		//Convert aligned cloud back into pointcloud2 message:
		aligned_cloud.header = header;
		aligned_cloud.header.frame_id = settings.aligned_cloud_frame;
		aligned_cloud.fields = fields;
		aligned_cloud.point_step = point_step;
		aligned_cloud.height = 1;
		aligned_cloud.width = cloud_in.size();
		aligned_cloud.row_step = aligned_cloud.point_step * cloud_in.size();
		
		aligned_cloud.data.resize(aligned_cloud.row_step);
		memcpy(&(aligned_cloud.data[0]), &(cloud_in[0]), aligned_cloud.row_step);
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	alignToGround
	 * @brief 		re-orients the given cloud so that the detected ground plane is level
	 * @param 		cloud_in - input cloud to align
	 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
	 * @param 		cloud - point vector version of the aligned cloud
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::alignToGround(sensor_msgs::msg::PointCloud2& cloud_in, sensor_msgs::msg::PointCloud2& aligned_cloud, std::vector<T>& cloud)
	{
		pointcloud_utils::convertFromPointCloud2(cloud_in, cloud);
		alignToGround(cloud, aligned_cloud, cloud);
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	alignToGround
	 * @brief 		re-orients the given cloud so that the detected ground plane is level
	 * @param 		aligned_cloud - pointcloud2 version of the aligned cloud
	 * @param 		cloud - point vector version of the aligned cloud
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::alignToGround(sensor_msgs::msg::PointCloud2& aligned_cloud, std::vector<T>& cloud)
	{
		std::vector<T> local_current_cloud = this->current_cloud;
		alignToGround(local_current_cloud, aligned_cloud, cloud);
	}

	//TODO: make a version of this that takes in a cloud from the outside
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
	template <class T> void GroundProcessor<T>::separateGround
	(
		sensor_msgs::msg::PointCloud2& ground_msg, 
		sensor_msgs::msg::PointCloud2& nonground_msg, 
		std::vector<T>& ground,
		std::vector<T>& nonground
	)
	{
		if (!plane_detected)
		{
			std::cout << "Warning! Ground has not been detected yet. Separate ground.\n";
			return;
		}

		std::vector<T> local_current_cloud = this->current_cloud; //save a local copy
		
		ground_msg.data.clear();
		nonground_msg.data.clear();
		ground.clear();
		nonground.clear();

		double slope1;
		double slope2;

		T pt;
		// T pt_prev = local_current_cloud[0];
		T pt_prev;
		pt_prev.x = 0;
		pt_prev.y = 0;
		pt_prev.z = 1 / plane_parameters.c_d;

		for ( uint i = 0; i < local_current_cloud.size(); i += settings.plane_parser_settings.point_skip_num)
		{

			pt = local_current_cloud[i];

			bool is_ego = (pt.x > settings.ego_x_min) && (pt.x < settings.ego_x_max) &&
						  (pt.y > settings.ego_y_min) && (pt.y < settings.ego_y_max) &&
						  (pt.z > settings.ego_z_min) && (pt.z < settings.ego_z_max);

			if (pt.x < settings.x_min || pt.x > settings.x_max ||
				pt.y < settings.y_min || pt.y > settings.y_max ||
				pt.z < settings.z_min || pt.z > settings.z_max ||
				is_ego
			   )
			{
				continue; //point is out of bounds
			}

			double plane_z = (- plane_parameters.a_d * pt.x - plane_parameters.b_d * pt.y + 1) / plane_parameters.c_d;
			if ( pt.z < (plane_z - settings.point_to_plane_tolerance) ) //TODO: add tolerance?
			{
				// std::cout << "Point is under ground\n";
				ground.push_back(pt);
				pt_prev = pt;
			}else if (plane_parser->pointIsOnPlane(pt, plane_parameters, settings.point_to_plane_tolerance) )
			{
				slope1 = abs(atan2((double)(pt_prev.z - pt.z), (double)(pt_prev.x - pt.x)) );
				slope2 = abs(atan2((double)(pt_prev.z - pt.z), (double)(pt_prev.y - pt.y)) );
				if (slope1 > M_PI/2) slope1 = M_PI - slope1;
				if (slope2 > M_PI/2) slope2 = M_PI - slope2;
				if ( ( slope1 < settings.pt_slope_threshold ) && ( slope2 < settings.pt_slope_threshold ) )
				{
					// std::cout << "Slope x: " << slope1 << " slope2: " << slope2 << "\n";
					// std::cout << "Threshold: " << settings.pt_slope_threshold << "\n";
					// std::cout << "Point is on ground\n";
					ground.push_back(pt);
					pt_prev = pt;
				}

			} else
			{
				nonground.push_back(pt);
			}
		}

		//Convert clouds back to pointcloud messages:
		ground_msg.header = header;
		ground_msg.fields = fields;
		ground_msg.point_step = point_step;
		ground_msg.height = 1;
		ground_msg.width = ground.size();
		ground_msg.row_step = ground_msg.point_step * ground.size();
		
		ground_msg.data.resize(ground_msg.row_step);
		memcpy(&(ground_msg.data[0]), &(ground[0]), ground_msg.row_step);


		nonground_msg.header = header;
		nonground_msg.fields = fields;
		nonground_msg.point_step = point_step;
		nonground_msg.height = 1;
		nonground_msg.width = nonground.size();
		nonground_msg.row_step = nonground_msg.point_step * nonground.size();
		
		nonground_msg.data.resize(nonground_msg.row_step);
		memcpy(&(nonground_msg.data[0]), &(nonground[0]), nonground_msg.row_step);
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	transformCloud
	 * @brief 		transforms the given cloud by the given transform
	 * @param 		cloud - points to transform
	 * @param 		transform - transform to use
	 * @return 		void
	 */
	template <class T> void GroundProcessor<T>::transformCloud(std::vector<T>& cloud, Eigen::Matrix4f& transform)
	{
		//TODO: fix matrix dimensions
		//TODO: confirm transform

		Eigen::MatrixXf point_matrix(4, cloud.size());

		int i = 0;
		for (T pt : cloud)
		{
			point_matrix.col(i) << pt.x, pt.y, pt.z, 1;
			i++;
		}

		Eigen::MatrixXf transformed_matrix(4, cloud.size());
		transformed_matrix = transform * point_matrix;

		for (uint i = 0; i < cloud.size(); i++)
		{
			 cloud[i].x = transformed_matrix.col(i)[0];
			 cloud[i].y = transformed_matrix.col(i)[1];
			 cloud[i].z = transformed_matrix.col(i)[2];
		}
	}

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
	template <class T> void GroundProcessor<T>::detectGround(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in, std::vector<T>& cloud, plane_parser_utils::PlaneParameters& plane_parameters, plane_parser_utils::States& plane_states)
	{
		sensor_msgs::msg::PointCloud2 parsed_cloud;
		// bool continue_from_last_plane = true;

		//std::cout << "Detecting ground! points: " << cloud_in->width << "\n";

		plane_parser->parsePlane( cloud_in, cloud, parsed_cloud, plane_parameters, plane_states, settings.plane_search_window, settings.intensity_min, settings.intensity_max);
		
		//std::cout << "Detected " << parsed_cloud.width << " plane points!\n";
		//std::cout << "Plane coefficients: " << plane_parameters.a_d << ", " << plane_parameters.b_d << ", " << plane_parameters.c_d << "\n";
		//std::cout << "States roll and pitch: " << plane_states.roll << ", " << plane_states.pitch << "\n";
	}

} //end namespace pointcloud_utils

#endif //GROUND_PROCESSOR_IMPL_HPP