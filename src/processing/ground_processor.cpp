/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2020
 * Brief: defines a class that detects and prcesses ground. Functions include removal or 
 *        segmentation of ground points and alignment of scan to ground
 * File: ground_processor.cpp
 */

// --------------------------
#include <pointcloud_utils/processing/ground_processor.hpp>
#include <pointcloud_utils/pointcloud_utils_impl.hpp> //the users don't need this!
// --------------------------

//TODO: add an option to remove points only if the intensity is in tolerance of the average for the selected window
//TODO: for some reason, if alignment is called, it changes the results of the separation algorithm

namespace pointcloud_utils
{
	GroundProcessor::GroundProcessor(GroundProcessor::Settings& settings)
	{
		plane_detected = false;
		this->settings = settings;
		this->plane_parser = new pointcloud_utils::PlaneParser(settings.plane_parser_settings);
	}

	GroundProcessor::~GroundProcessor()
	{
		delete plane_parser;
	}

	void GroundProcessor::updateSettings(GroundProcessor::Settings& settings)
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
	void GroundProcessor::updateCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud)
	{
		//std::cout << "Points in: " << cloud_in->width << "\n";
		plane_detected = false;

		header = cloud_in->header;
		fields = cloud_in->fields;
		point_step = cloud_in->point_step;

		//detect ground
		PlaneParser::PlaneParameters plane_parameters;
		PlaneParser::States plane_states;
		//std::cout << "Detecting ground\n";
		detectGround(cloud_in, cloud, plane_parameters, plane_states);
		//std::cout << "Done detecting ground\n";
		this->current_cloud = cloud;
		this->plane_parameters = plane_parameters;
		this->plane_states = plane_states;
		//std::cout << "Plane coefficients: " << this->plane_parameters.a_d << ", " << this->plane_parameters.b_d << ", " << this->plane_parameters.c_d << "\n";
		
		//std::cout << "States roll and pitch: " << this->plane_states.roll << ", " << this->plane_states.pitch << "\n";

		plane_detected = true;
	}

	/**
	 * @function 	returnPlaneDescriptors
	 * @brief 		sends back the detected plane coefficients
	 * @param 		plane_parameters - place to store the found plane parameters
	 * @param 		plane_states - place to store the found plane states
	 * @return 		void
	 */
	void GroundProcessor::returnPlaneDescriptors(PlaneParser::PlaneParameters& plane_parameters, PlaneParser::States& plane_states)
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
	void GroundProcessor::alignToGround(std::vector<pointcloud_utils::pointstruct>& cloud_in, sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud)
	{
		if (!plane_detected)
		{
			std::cout << "Warning! Ground has not been detected yet. Cannot align to ground.\n";
			return;
		}

		//Save local copies
		PlaneParser::States local_plane_states = this->plane_states;

		std::cout << "Roll, pitch: " << local_plane_states.roll << ", " << local_plane_states.pitch << "\n";
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
	void GroundProcessor::alignToGround(sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud)
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
	void GroundProcessor::alignToGround(sensor_msgs::PointCloud2& aligned_cloud, std::vector<pointcloud_utils::pointstruct>& cloud)
	{
		std::vector<pointcloud_utils::pointstruct> local_current_cloud = this->current_cloud;
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
	void GroundProcessor::separateGround
	(
		sensor_msgs::PointCloud2& ground_msg, 
		sensor_msgs::PointCloud2& nonground_msg, 
		std::vector<pointcloud_utils::pointstruct>& ground,
		std::vector<pointcloud_utils::pointstruct>& nonground
	)
	{
		if (!plane_detected)
		{
			std::cout << "Warning! Ground has not been detected yet. Separate ground.\n";
			return;
		}

		std::vector<pointcloud_utils::pointstruct> local_current_cloud = this->current_cloud; //save a local copy
		
		ground_msg.data.clear();
		nonground_msg.data.clear();
		ground.clear();
		nonground.clear();

		for (pointcloud_utils::pointstruct pt : local_current_cloud )
		{
			if (plane_parser->pointIsOnPlane(pt, plane_parameters, settings.point_to_plane_tolerance))
			{
				ground.push_back(pt);
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
	void GroundProcessor::transformCloud(std::vector<pointcloud_utils::pointstruct>& cloud, Eigen::Matrix4f& transform)
	{
		//TODO: fix matrix dimensions
		//TODO: confirm transform

		Eigen::MatrixXf point_matrix(4, cloud.size());

		int i = 0;
		for (pointcloud_utils::pointstruct pt : cloud)
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
	void GroundProcessor::detectGround(const sensor_msgs::PointCloud2::ConstPtr& cloud_in, std::vector<pointcloud_utils::pointstruct>& cloud, PlaneParser::PlaneParameters& plane_parameters, PlaneParser::States& plane_states)
	{
		sensor_msgs::PointCloud2 parsed_cloud;
		bool continue_from_last_plane = true;

		//std::cout << "Detecting ground! points: " << cloud_in->width << "\n";

		plane_parser->parsePlane( cloud_in, cloud, parsed_cloud, plane_parameters, plane_states, settings.plane_search_window, settings.intensity_min, settings.intensity_max);
		
		//std::cout << "Detected " << parsed_cloud.width << " plane points!\n";
		//std::cout << "Plane coefficients: " << plane_parameters.a_d << ", " << plane_parameters.b_d << ", " << plane_parameters.c_d << "\n";
		//std::cout << "States roll and pitch: " << plane_states.roll << ", " << plane_states.pitch << "\n";
	}

} //end namespace pointcloud_utils