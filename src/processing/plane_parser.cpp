/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 8 Sept 2020
 * Brief: class that does a plane fit to the points in the given window, and reports the motion of the fitted plane
 * File: plane_parser.cpp
 */

// --------------------------
#include "pointcloud_utils/processing/plane_parser.hpp"
#include <pointcloud_utils/pointcloud_utils_impl.hpp> //the users don't need this!
// --------------------------

namespace pointcloud_utils
{
	PlaneParser::PlaneParser(const PlaneParser::Settings& settings)
	{
		this->settings = settings;
	}

	PlaneParser::~PlaneParser()
	{

	}

	void PlaneParser::updateSettings(PlaneParser::Settings& settings)
	{
		this->settings = settings;
	}

	/**
	 * @function 	parsePlane
	 * @brief 		finds the plane-fit of the given cloud, using the given window bounds
	 * @param 		cloud_in - inputted 3D point cloud
	 * @param 		cloud - space to store converted 3D cloud
	 * @param 		plane_points  - space to store the filtered plane points
	 * @param 		plane_parameters - coefficients of the fitted plane equation
	 * @param 		plane_states - values representing planar position and orientation
	 * @param 		search_window - window within which to process points for this plane
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @return 		void
	 */
	void PlaneParser::parsePlane
	(
		const sensor_msgs::PointCloud2::ConstPtr& cloud_in, 
		std::vector<pointcloud_utils::pointstruct>& cloud, 
		sensor_msgs::PointCloud2& filtered_cloud,
		PlaneParser::PlaneParameters& plane_parameters,
		PlaneParser::States& plane_states,
		pointcloud_utils::SearchWindow& search_window,
		bool continue_from_last_plane, 
		const float intensity_min,
		const float intensity_max
	)
	{
		if (cloud.size() < settings.min_points_to_fit)
		{
			std::cout << "Not enough points to fit plane in parseplane: " << cloud.size() << "\n";
			return;
		}
		//Initialize the covariance matrix:
		plane_parameters.covariance_matrix = Eigen::Matrix3f::Zero();
		// convert to point vector
		pointcloud_utils::convertFromPointCloud2(cloud_in, cloud);
		
		std::vector<pointcloud_utils::pointstruct> filtered_cloud_vector;

		this->parsePlane
		(
			cloud, 
			filtered_cloud_vector,
			plane_parameters, 
			plane_states, 
			search_window, 
			continue_from_last_plane,
			cloud_in->header.stamp.toSec(),
			intensity_min,
			intensity_max
		);

		//convert back to pointcloud message:
		filtered_cloud.header = cloud_in->header;
		filtered_cloud.fields = cloud_in->fields;
		filtered_cloud.point_step = cloud_in->point_step;
		filtered_cloud.height = 1;
		filtered_cloud.width = filtered_cloud_vector.size();
		filtered_cloud.row_step = filtered_cloud.point_step * filtered_cloud_vector.size();
			
		if (settings.do_transform)
		{
			filtered_cloud.header.frame_id = settings.transform_frame;
		}
		//std::cout << "Plane Coefficients: " << plane_parameters.a_d << ", so on\n";
		//std::cout << "Plane states: " << plane_states.x << ", so on\n";

		filtered_cloud.data.resize(filtered_cloud.row_step);
		memcpy(&(filtered_cloud.data[0]), &(filtered_cloud_vector[0]), filtered_cloud.row_step);
	
	}
	/**
	 * @function 	parsePlane
	 * @brief 		finds the plane-fit of the given cloud, using the given window bounds
	 * @param 		cloud - inputted 3D point cloud
	 * @param 		plane_points  - space to store the filtered plane points
	 * @param 		plane_parameters - coefficients of the fitted plane equation
	 * @param 		plane_states - values representing planar position and orientation
	 * @param 		time - rostime in seconds for this cloud
	 * @param 		search_window - window within which to process points for this plane
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @return 		void
	 */
	void PlaneParser::parsePlane
	(
		std::vector<pointcloud_utils::pointstruct>& cloud,
		std::vector<pointcloud_utils::pointstruct>& filtered_cloud,
		PlaneParser::PlaneParameters& plane_parameters,
		PlaneParser::States& plane_states,
		pointcloud_utils::SearchWindow& search_window,
		bool continue_from_last_plane, 
		const double time,
		const float intensity_min,
		const float intensity_max
	)
	{
		if (continue_from_last_plane)
		{
			if (first)
			{
				//first = false;
				last_state_time = this_state_time;
			}
			this_state_time = time;
		}


		if (settings.do_transform)
		{
			pointcloud_utils::transformCloud(cloud, settings.transform);
		}

		//plane parsing!
		std::vector<pointcloud_utils::pointstruct> cloud_parsed;
		PlaneParser::LeastSquaresMatricies least_squares_matricies;
		//std::cout << "Entering plane finding\n";
		if (!findPlane( cloud, cloud_parsed, search_window, least_squares_matricies, plane_states, intensity_min, intensity_max))
		{
			return;
		}

		//std::cout << "Finished plane finding\n";
		if (least_squares_matricies.plane_coefficients.size() != 3)
		{
			plane_parameters.a_d = 0;
			plane_parameters.b_d = 0;
			plane_parameters.c_d = 0;
		} else
		{
			plane_parameters.a_d = least_squares_matricies.plane_coefficients[0];
			plane_parameters.b_d = least_squares_matricies.plane_coefficients[1];
			plane_parameters.c_d = least_squares_matricies.plane_coefficients[2];
		}

		getCovariance(plane_parameters, least_squares_matricies);

		getPlaneStates( least_squares_matricies.plane_coefficients, plane_parameters.covariance_matrix, plane_states, search_window, continue_from_last_plane);

		//std::cout << "Finished getting plane states\n";

		if (continue_from_last_plane)
		{
			if (first)
			{
				first = false;
				//Initialize filter:
			}
		}

		//std::cout << "Points found: " << cloud_parsed.size() << "\n";

		filtered_cloud = cloud_parsed;
	}

	//TODO: make a templated version which requires the specification of a pointstruct type
	/**
	 * @function 	pointIsOnPlane
	 * @brief 		checks if the given point is on the given plane or not, using the given tolerance
	 * @param 		pt - point to check
	 * @param 		plane_parameters - description of plane to check on
	 * @param 		tolerance - threshold for distance from plane
	 * @return 		bool - true if the point is on the plane, 
	 * 				       false otherwise
	 */
	bool PlaneParser::pointIsOnPlane(const pointcloud_utils::pointstruct& pt, const PlaneParser::PlaneParameters& plane_parameters, const float& tolerance)
	{
		//Distance to plane from origin: https://mathinsight.org/distance_point_plane
		float normal_magnitude = std::sqrt(std::pow(plane_parameters.a_d, 2) + std::pow(plane_parameters.b_d, 2) + std::pow(plane_parameters.c_d, 2) );
		float distance = (plane_parameters.a_d * pt.x + plane_parameters.b_d * pt.y + plane_parameters.c_d * pt.z - 1 )/ normal_magnitude;
		
		return(pointcloud_utils::inTolerance(distance, 0, tolerance));
	}

	/**
	 * @Function 	findPlane
	 * @Param 		cloud - point cloud to parse
	 * @Param 		plane_points - place to save parsed cloud
	 * @Param 		search_window - bounds within which to process points
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @param 		matricies - struct with eigen matricies used in the least squares solution (to be generated by this function)
	 * @param 		plane_states - values representing planar position and orientation
	 * @param 		intensity_min - minimum intensity to accept into plane (default 0)
	 * @param 		intensity_max - maximum intensity to accept into plane (default 256)
	 * @Return 		bool true if successful
	 * 					 false otherwise
	 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
	 */
	bool PlaneParser::findPlane
	(
		std::vector<pointcloud_utils::pointstruct>& cloud, 
		std::vector<pointcloud_utils::pointstruct>& plane_points, 
		const pointcloud_utils::SearchWindow& search_window,
		PlaneParser::LeastSquaresMatricies& matricies,
		PlaneParser::States& plane_states,
		const float intensity_min,
		const float intensity_max
	)
	{
		//Find points of interest
	
		//std::cout << "Finding plane\n";
	
		bool pt_1_found = false;
		bool pt_2_found = false;
		bool pt_3_found = false;
	
		bool track_point_found = false;
	
		//std::vector<pointcloud_utils::pointstruct> plane_points;
	
		pointcloud_utils::pointstruct pt1;
		pointcloud_utils::pointstruct pt2;
		pointcloud_utils::pointstruct pt3;
	
		pointcloud_utils::pointstruct track_pt;
	
		pointcloud_utils::pointstruct track_pt_x;
		pointcloud_utils::pointstruct track_pt_y;
		pointcloud_utils::pointstruct track_pt_z;
	
		//std::cout << "bounds: " << x_max << ", " << x_min << ", " << y_max << ", " << y_min
		//						<< ", " << z_max << ", " << z_min << "\n";
		
		//std::cout << "bounds: " << search_window.x_max << ", " << search_window.x_min << ", "
		//						<< search_window.y_max << ", " << search_window.y_min << ", "
		//						<< search_window.z_max << ", " << search_window.z_min << ", "
		//						<< intensity_max 	   << ", " << intensity_min << "\n";

		//for each point in the cloud, push it to the cloud_parsed if it is not filtered out
		//std::cout << "Cloud size: " << cloud.size() << "\n";
		for (pointcloud_utils::pointstruct pt : cloud)
		{
			if (!settings.use_point_track_method)
			{
				//std::cout << "Point: " << pt.x << ", " << pt.y << ", " << pt.z << ", " << pt.intensity << "\n";
				if (pt.x <= search_window.x_max && pt.x >= search_window.x_min &&
					pt.y <= search_window.y_max && pt.y >= search_window.y_min &&
					pt.z <= search_window.z_max && pt.z >= search_window.z_min &&
					pt.intensity <= intensity_max && pt.intensity >= intensity_min)
				{
					plane_points.push_back(pt);
					//std::cout << "Found point!\n";
				}
			} else
			{
				std::cout << "Warning! Point track method has not yet been implemented\n";
				//TODO: select 3 specific points in the window to fit a plane to, and one to use as the track point
			}
		}
	
		//std::cout << "Initial plane points found: " << plane_points.size() << "\n";
	
		if (!settings.use_point_track_method)
		{
			if (plane_points.size() < settings.min_points_to_fit)
			{
				std::cout << "Not enough points to fit plane in findPlane: " << plane_points.size() << "\n";
				
				return false;
			}
			//std::cout << "Found ground points: " << ground_points.size() << "\n";
			
			//Plane fit over the ground point cluster!
			//std::cout << "entering plane fit method\n";
			fitPlane(plane_points, matricies);
			//std::cout << "Fitted plane\n";
			if (settings.iterate_plane_fit)
			{
				//std::cout << "removing outliers\n";
				int plane_fit_iterations = 1;
				int outliers_removed = removeOutliers(plane_points, matricies.plane_coefficients);
				while (plane_fit_iterations < settings.max_iterations && outliers_removed > 0)
				{
					fitPlane(plane_points, matricies);
					outliers_removed = removeOutliers(plane_points, matricies.plane_coefficients);
					plane_fit_iterations++;
				}
			}
		} else
		{
			//TODO: point track method??
		}

		return true;
	}

	/**
	 * @Function 	fitPlane
	 * @Param 		cloud - points to fit a plane to
	 * @Param 		matricies - saves the matricies used in the least squares soltion 
	 * @Return 		void
 	 * @Brief 		Fits a plane equation (a/d * x + b/d * y + c/d * z = 1) to the given point cloud
	 * //TODO: what happens when my plane has d = 0?? (intersects the origin) - I guess the coefficients will approach infinity
	 * //TODO: look into using robust regression to support outlier tolerance, and brainstorm other techniques for increasing plaraity of underlying points for each fit
	 */
	void PlaneParser::fitPlane(const std::vector<pointcloud_utils::pointstruct>& cloud, PlaneParser::LeastSquaresMatricies& matricies)
	{
		//std::cout << "Fitting plane to " << cloud.size() << " points\n";
		if (cloud.size() < settings.min_points_to_fit)
		{
			std::cout << "Not enough points to fit plane in fitplane: " << cloud.size() << "\n";
			return;
		}
		//Plane fit over the ground point cluster!
		// Plane equation: ax + by + cz = d
		// Least squares generic solvable form: Ax = b
		// Plane ls solvable form: [x y z] * [a/d b/d c/d]^T = 1
		
		//Populate points_matrix, the A matrix:
		int i = 0;
		//Eigen::MatrixXf points_matrix = Eigen::MatrixXf::Zero(cloud.size(), 3);
		matricies.points_matrix = Eigen::MatrixXf::Zero(cloud.size(), 3);
		for (pointcloud_utils::pointstruct pt : cloud)
		{
			matricies.points_matrix.block<1,3>(i,0) << pt.x, pt.y, pt.z;
			i++;
		}
		//std::cout << "\n Matrix: " << points_matrix << "\n";
		
		//Create plane_coefficeints, the x vector:
		//Eigen::Vector3f plane_coefficients = Eigen::Vector3f::Zero();
		matricies.plane_coefficients = Eigen::Vector3f::Zero();
		
		//Populate sum_vector, the b vector:
		//Eigen::VectorXf sum_vector = Eigen::VectorXf::Constant(cloud.size(), 1, 1);
		matricies.sum_vector = Eigen::VectorXf::Constant(cloud.size(), 1, 1);
	
		//Solve least squares:
		if (settings.plane_fit_type == pointcloud_utils::PlaneParser::PlaneFitTypes::SIMPLE)
		{
			matricies.plane_coefficients = (matricies.points_matrix.transpose() * matricies.points_matrix).inverse() * matricies.points_matrix.transpose() * matricies.sum_vector;
		}
		else if (settings.plane_fit_type == pointlcoud_utils::PlaneParser::PlaneFitTypes::SVD)
		{
			matricies.plane_coefficients = matricies.points_matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matricies.sum_vector);
		} else
		{
			std::cout << "Plane fit option " << settings.plane_fit_type << " not supported. Zeroing plane coefficient results\n";
			matricies.plane_coefficients << 0, 0, 0;
		return;
	}

	/**
	 * @Function 	removeOutliers
	 * @Param 		plane_points - points belonging to the current plane
	 * @Param 		plane_coefficients - parameters of the current plane fit
	 * @Reutnr 		int - number of outliers removed
	 * @Brief 		Removes outliers that are out of tolerance of the given plane equation
	 */
	int PlaneParser::removeOutliers(std::vector<pointcloud_utils::pointstruct>& plane_points, const Eigen::Vector3f& plane_coefficients)
	{
		std::vector<pointcloud_utils::pointstruct> saved_points;
		saved_points = plane_points; //TODO: make sure this isn't a linked reference
		plane_points.clear();

		int outlier_count = 0;
		for (pointcloud_utils::pointstruct pt : saved_points)
		{
			////Solve for pt.x distance to plane:
			//// a/d x + b/d y + c/d z = 1 -> x = (1 - b/d y - c/d z) / (a/d)
			//float x_plane = (1 - plane_coefficients[1] * pt.y - plane_coefficients[2] * pt.z) / plane_coefficients[0];
			//if (!pointcloud_utils::inTolerance(x_plane, pt.x, settings.outlier_tolerance))
			//{
			//	outlier_count++;
			//	continue; //Skip point
			//}
//
			////Solve for pt.y distance to plane:
			//float y_plane = (1 - plane_coefficients[0] * pt.x - plane_coefficients[2] * pt.z) / plane_coefficients[1];
			//if (!pointcloud_utils::inTolerance(y_plane, pt.y, settings.outlier_tolerance))
			//{
			//	outlier_count++;
			//	continue; //Skip point
			//}
//
			////Solve for pt.z distance to plane:
			//float z_plane = (1 - plane_coefficients[0] * pt.x - plane_coefficients[1] * pt.y) / plane_coefficients[2];
			//if (!pointcloud_utils::inTolerance(z_plane, pt.z, settings.outlier_tolerance))
			//{
			//	outlier_count++;
			//	continue; //Skip point
			//}

			PlaneParser::PlaneParameters params;
			params.a_d = plane_coefficients[0];
			params.b_d = plane_coefficients[1];
			params.c_d = plane_coefficients[2];


			if (pointIsOnPlane(pt, params, settings.outlier_tolerance))
			{
				plane_points.push_back(pt);
			} else
			{
				outlier_count++;
			}

		}

		return outlier_count;
	}

	/**
	 * @Function 	getPlaneStates
	 * @param 		plane_coefficients - coefficients of the fitted plane equation, with variance
	 * @param 		plane_parameters_covariance - vector holding row-major-order 3x3 covariance matrix for plane parameters a/d, b/d, c/d
	 * @param 		plane_states - values representing planar position and orientation (to be found)
	 * @Param 		search_window - bounds within which to process points
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @Return 		void
	 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
	 */
	void PlaneParser::getPlaneStates
	(
		const Eigen::Vector3f& plane_coefficients,
		const Eigen::Matrix3f& plane_parameters_covariance,
		PlaneParser::States& plane_states,
		const pointcloud_utils::SearchWindow& search_window,
		bool continue_from_last_plane
	)
	{

		plane_states.covariance_matrix = Eigen::MatrixXf::Zero(12, 12);
				

		if (plane_coefficients[0] == 0 && plane_coefficients[1] == 0 && plane_coefficients[2] == 0)
		{
			//Bad plane fit! return zero's
			plane_states.x = 0;
			plane_states.y = 0;
			plane_states.z = 0;
			plane_states.roll = 0;
			plane_states.pitch = 0;
			plane_states.yaw = 0;
		} else if (!settings.use_point_track_method)
		{
			// // do this at center of rotation instead
			float center_x = (search_window.x_max + search_window.x_min) / 2;
			float center_y = (search_window.y_max + search_window.y_min) / 2;
			float center_z = (search_window.z_max + search_window.z_min) / 2;
	// 
			// //Get planar translations in all directions:
			// float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
			// float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
			// float d_prime = (1/plane_coefficients[0]);
			// 
			// track_pt_x.y = center_y;
			// track_pt_x.z = center_z;
			// track_pt_x.x = b_prime * track_pt.y + c_prime * track_pt.z + d_prime;
		// 
			// float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
			// c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
			// d_prime = (1/plane_coefficients[1]);
			// 
			// track_pt_y.x = center_x;
			// track_pt_y.z = center_z;
			// track_pt_y.y = a_prime * track_pt.x + c_prime * track_pt.z + d_prime;
			// 
			// a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
			// b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
			// d_prime = (1/plane_coefficients[2]);
			// track_pt_z.x = center_x;
			// track_pt_z.y = center_y;
			// track_pt_z.z = a_prime * track_pt.x + b_prime * track_pt.y + d_prime;
			Eigen::Matrix3f translation_covariance = Eigen::Matrix3f::Zero();
			Eigen::Matrix3f orientation_covariance = Eigen::Matrix3f::Zero();
			Eigen::Matrix3f translation_rate_covariance = Eigen::Matrix3f::Zero();
			Eigen::Matrix3f rotation_rate_covariance = Eigen::Matrix3f::Zero();

			//Get translation at sensor origin!!
			pointcloud_utils::pointstruct origin_pt; //NOTE: this is designed to work best for a nearly horizontal plane
			
			if (settings.report_offsets_at_origin)
			{
				//Offsets to plane at origin:
				origin_pt.x = 1 / plane_coefficients[0];
				origin_pt.y = 1 / plane_coefficients[1];
				origin_pt.z = 1 / plane_coefficients[2];

				//TODO: confirm error propogation: 
				//translation_covariance = plane_parameters_covariance.inverse(); //TODO: I am not convinced of this logic. Let's just use the plane parameters covariance?

				translation_covariance = plane_parameters_covariance;

				if (std::isnan(origin_pt.x) || std::isinf(origin_pt.x)) origin_pt.x = 0;
				if (std::isnan(origin_pt.y) || std::isinf(origin_pt.y)) origin_pt.y = 0;				
				if (std::isnan(origin_pt.z) || std::isinf(origin_pt.z)) origin_pt.z = 0;
			} else
			{
				//Distance to plane from origin: https://mathinsight.org/distance_point_plane
				float normal_magnitude = std::sqrt(std::pow(plane_coefficients[0], 2) + std::pow(plane_coefficients[1], 2) + std::pow(plane_coefficients[2], 2) );
				float distance = 1 / normal_magnitude;
				//This distance is in the direction of the planar normal vector, [a, b, c]:
				// sqrt(x2 + y2 + z2) = distance
				// m [ a, b c] = [x, y, z]
				// distance = sqrt(ma2 + mb2 + mc2) = sqrt(m) * (norm of normal)
				// distance/norm_of_normal = sqrt(m)
				// m = (distance/norm_of_normal)^2
		
				float scale = std::pow((distance / normal_magnitude), 2);
		
				//std::cout << "Scale: " << scale << "\n";
		
				origin_pt.x = scale * plane_coefficients[0];
				origin_pt.y = scale * plane_coefficients[1];
				origin_pt.z = scale * plane_coefficients[2];

				translation_covariance = scale * plane_parameters_covariance;
			}

			//if (std::isinf(origin_pt.x)) origin_pt.x = 0;
			//if (std::isinf(origin_pt.y)) origin_pt.y = 0;
			//if (std::isinf(origin_pt.z)) origin_pt.z = 0;
			//TODO: how to do vel tracking if is inf?
		
			//std::cout << "origin pt: " << origin_pt.x << "\n";
			//Get orientation of the plane:
			double roll, pitch, yaw;
			Eigen::Quaternion<float> quat;
			getPlaneOrientation(plane_coefficients, orientation_covariance, roll, pitch, yaw, quat);
			double elapsed_time = (this_state_time - last_state_time);
			if (elapsed_time != 0 && continue_from_last_plane) //TODO: This may not work properly with quats yet
			{
				//std::cout << "Elapsed time: " << elapsed_time << "\n";
				//Find relative motion of track point:
				plane_states.roll_vel = (roll - plane_states.roll) / elapsed_time;
				plane_states.pitch_vel = (pitch - plane_states.pitch) / elapsed_time;
				plane_states.yaw_vel = (yaw - plane_states.yaw_vel) / elapsed_time;

				rotation_rate_covariance = orientation_covariance; //TODO: not convinced of this

				//std::cout << "Elapsed time: " << elapsed_time << "\n";
				//Find relative motion of track point:
				//plane_states.x_vel = (track_pt_x.x - tracked_plane_states.x) / elapsed_time;
				//plane_states.y_vel = (track_pt_y.y - tracked_plane_states.y) / elapsed_time;
				//plane_states.z_vel = (track_pt_z.z - tracked_plane_states.z) / elapsed_time;
				
				plane_states.x_vel = (origin_pt.x - tracked_plane_states.x) / elapsed_time;
				plane_states.y_vel = (origin_pt.y - tracked_plane_states.y) / elapsed_time;
				plane_states.z_vel = (origin_pt.z - tracked_plane_states.z) / elapsed_time;

				translation_rate_covariance = translation_covariance; //TODO: not convinced of this
			}

			// if goodness of fit, use the variance for all covariance values
			if (settings.covariance_type == PlaneParser::CovarianceType::GOODNESS_OF_FIT_ERROR ||
				settings.covariance_type == PlaneParser::CovarianceType::GOODNESS_OF_FIT_DISTANCE)
			{
				translation_covariance = Eigen::Matrix3f::Identity() * plane_parameters_covariance(0,0); //Set diagonals to the variance
				translation_rate_covariance = translation_covariance;
				orientation_covariance = translation_covariance;
				rotation_rate_covariance = translation_covariance;
			}
	
			plane_states.roll = roll;
			plane_states.pitch = pitch;
			plane_states.yaw = yaw;
	
			// plane_states.x = track_pt_x.x;
			// plane_states.y = track_pt_y.y;
			// plane_states.z = track_pt_z.z;

			plane_states.x = - origin_pt.x;
			plane_states.y = - origin_pt.y;
			plane_states.z = - origin_pt.z; //Note: This always gives me the center of the plane window to the sensor origin. 

			Eigen::Matrix3f zeros = Eigen::Matrix3f::Zero();
		
			//plane_states.variance = plane_coefficients[3];
			plane_states.variance = translation_covariance(0,0);
			Eigen::MatrixXf row1(3, 12);
			Eigen::MatrixXf row2(3, 12);
			Eigen::MatrixXf row3(3, 12);
			Eigen::MatrixXf row4(3, 12);

			row1 = translation_covariance, zeros, zeros, zeros;
			row2 = zeros, orientation_covariance, zeros, zeros;
			row3 = zeros, zeros, translation_rate_covariance, zeros;
			row4 = zeros, zeros, zeros, rotation_rate_covariance, zeros;

			plane_states.covariance_matrix = row1, row2, row3, row4; //TODO: check this initialization sequence

		} else
		{
			std::cout << "Warning! Point track method has not yet been implemented\n";
		}
	}

	
	/**
	 * @Function 	getPlaneOrientation
	 * @Param 		plane_coefficients - vector of plane equation coefficients, a/d, b/d, c/d, with variance
	 * @param 		orientation_covariance - covariance for the found angles
	 * @param 		roll - roll angle of the plane (to be found, if requested)
	 * @param 		pitch - pitch angle of the plane (to be found, if requested)
	 * @param 		yaw - yaw angle of the plane (to be found, if requested)
	 * @param 		quat - quaternion of the plane orienation (to be found, if requested)
	 * @Return 		void
	 * @Brief 		determines the orientation of the plane described by the given plane equation
	 */
	void PlaneParser::getPlaneOrientation(const Eigen::Vector3f& plane_coefficients, Eigen::Matrix3f& orientation_covariance, double& roll, double& pitch, double& yaw, Eigen::Quaternion<float>& quat)
	{
		//TODO: get quaternion and break down into roll, pitch, yaw instead?

		//[notes:
		//TODO: move these thoughts into a word document:
		//Note: ratio of plane coefficient to the magnitude of a, b, and c together reflects the amount the plane is aligned to that axis.
			//      The smaller the ratio for a given coefficient, the greater the alignment to that axis - so, the ratio might be proportional to an angle displacement from that axis!!
			// For line intersecting the origin plane(x, y): a / ||a,b|| * 90' = angle offset from x axis (a/d * 1/(sqrt(a2/d2 + b2/d2)) = a/d * 1/(sqrt((a2 + b2)/d2)) = a/d * d/(sqrt((a2 + b2)) = a / ||a, b|| -- doesn't matter if thsee are not the pure coefficients!
			// For line intersecting the origin plane(y, z): c / ||b,c|| * 90' = angle offset from z axis
			// For line intersecting the origin plane(x, z): c / ||a,c|| * 90' = angle offset from z axis
			//** as both parameters for a plane approach zero, the angle solution approaches 1 (even though the plane is becoming more aligned with both directions, and should go to 0)
			//** Note that the percentage can never reach 90 so long as one of the coefficents for that origin plane is non-zero -- the plane will not align with that plane until the driving coefficients are both zero
			//**If a denominator is zero, the result is zero (the numerator zero dominates)
			//Orientation can be defined by the intersection line orientations in each of the three planes!

			//Can we convert this to euler angles?
			// If z angular offset is zero (c is approximately zero, and a and b are not approximately zero), then the plane is vertical. 
			//      It has a nearly pure yaw about the z axis, equal to the angular offset from the desired axis in the x, y plane. Its pitch and roll are ambiguous - a rotation of either one or both 90' would explain the vertical nature of the plane,
			//      as would any number of combinations of pitches and rolls. We can define a convention, where one is defined first, and whatever orientation that is left is attributed to the remaining one. Therefore, if roll came first, the vertical plane
			//      has a roll of 90' and a pitch of 0' (or otherwise undefined). If one of the two is known from an external measure, the remaining one can be solved directly to provide the remaining orientation.
			// 		-- How does this relate to the plane ratios?
			// If y is zero, plane has pure pitch
			// If x is zero, plane has pure roll
			// -- Note: plane coefficients describe the normal vector, independent of d 
			// normalizing the normal vector can yield a simplified plane equation - divide the current d (1) by the magnitude of the normal vector as well, to keep the offset the same
			// Find x, y, z origin offsets (axis intersection points) by simply setting the other two axis values to zero (find x intercept by zeroing y and z and solving the plane equation for x)


			//Plane spin about the normal is ambiguous
			//Roll, pitch, yaw are ambiguous (non deterministic) without specified order
			// We can determine order by watching changes over time - integrate angular rates into orientation (this is deterministic, I think)

			//SO! we should probably try to represent the instantaneous orientation as a quaternion (can we do this quickly, from the plane equation?)
			// Find quaternion that explains the normal vector with respect to one of the axial planes -- let's choose horizontal axial plane (z = normal being standard)
			// - two normal vectors to a quaternion (here set reference normal to (0, 0, 1) for z axis) : https://math.stackexchange.com/questions/58305/quaternion-between-2-3d-planes
			//N1.normalize(), N2.normalize()
			//Vector M = N1+N2
			//M.normalize()
			//Vector axis = M.cross(N2)
			//angle = M.dot(N2)
			//Quaternion q(w=angle, x=axis.x, y=axis.y, z=axis)
			//q.normalize()


			//1x + 1 y  = 0 - horizontal plane (doesn't matter where z is)

			//1x + 1y + 0.001z = 0 -> plane at zero intersection pt, nearly flat
			//z = (n - x - y) / 0.001

			//1, 1 -> z = -2000 (must have really large value to drive the other two parameters to their values)
			//this means that, at 1,1, the plane has height of -2000! this isn't a shallow horizontal plane at all, but rather a vertical one
		//]
		

		//Note: for some reason, this seems to give angles that are 90', or pi/2 from truth (at least for a horizontal plane!)

		Eigen::Vector3f normal;
		normal[0] = plane_coefficients[0];
		normal[1] = plane_coefficients[1];
		normal[2] = plane_coefficients[2];

		//Normalize the normal vector:
		double norm = std::sqrt(std::pow(normal[0], 2) + std::pow(normal[1], 2) + std::pow(normal[2], 2));
		normal[0] = normal[0]/norm;
		normal[1] = normal[1]/norm;
		normal[2] = normal[2]/norm;

		//std::cout << "Normal: " << normal[0] << ", " << normal[1] << ", " << normal[2] << "\n";
	
		//TODO: filter normal, and keep it from randomly inverting direction
		//if (z_positive)
		//{
		// /	if (normal[2] < 0)
		// /	{
		// /		normal *= -1;
		// /	}
		// /	if (normal[2] == 0)
		// /	{
		// /		//TODO: maintain a smooth motion of normal as we move past axes
		// /	}
		//}

		//Covariance linearization: http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf for atan, https://socratic.org/questions/how-do-you-find-the-linearization-of-y-sin-1x-at-a-1-4 for asin
		// NOTE: neither atan nor asin are nicely linearized, so linearization for error propagation is unreasonable to attempt, if not impossible

		switch (settings.angle_solution_type)
		{
			case (PlaneParser::AngleSolutionType::ATTITUDE_ANGLES):
			{
				//World Roll, Pitch, Yaw: 
				// The axis of reference for each angle is treated as convention and must be known (this treats a horizontal plane as the default)
				// Note: these are the angles of plane lines which intersect the world frame cardinal planes. They do not represent euler roll, pitch, yaw angles - one of the angles is entirely explained by the other two? We can choose which one to ignore (I usually choose yaw)?? - plus an arbitrary amount of in-plane yaw?
				roll  = - std::atan2(normal[1], normal[2]); //angle of plane normal about world x axis, 0 at z axis (horizontal)
				pitch =   std::atan2(normal[0], normal[2]); //angle of plane normal about world y axis, 0 at z axis (horizontal)
				yaw   = - std::atan2(normal[1], normal[0]); //angle about world z axis, 0 at x axis (forward)
				//roll = std::atan2(normal[2], normal[1])	; //angle about world x axis, 0 at y axis (horizontal)
				//pitch = std::atan2(normal[2], normal[0]); //angle about world y axis, 0 at x axis (horizontal)
				//yaw = std::atan2(normal[1], normal[0])	; //angle about world z axis, 0 at x axis (forward)
				
				//Maybe these aren't rolls, pitches, etc that can be applied, but rather are instantaneous ones?
			
				//Attitudes, such as from GPS, seem to be interested in projections. if projected onto a horizontal plane, how yawed is the forward vector? This is local planar yaw
					// Pitch and roll are reported to horizontal reference plane somehow
					//https://novatel.com/solutions/attitude
					//"Attitude is the angular difference measured between an airplane’s axis and the line of the Earth’s horizon. Pitch attitude is the angle formed by the longitudinal axis, and bank attitude is the angle formed by the lateral axis."
					// https://en.wikiversity.org/wiki/Aircraft_piloting/Attitude_flying
						//This finds pitch, roll according to a certain planar yaw
		
				//These report pitch, roll angles of the plane to the axes. These are NOT Euler angles??. one of the three angles is zero (usually yaw)
		
				break;
			}
			case (PlaneParser::AngleSolutionType::EULER_ANGLES):
			{
				// //TODO: define YPR as the order, and solve for the successive rotations (yaw is zero here)
				// yaw = 0;
				// pitch = std::atan2(normal[0], normal[2]); //angle of plane normal about world y axis, 0 at z axis (horizontal)
				// //TODO: find roll about new x axis
				// //Rotate normal by pitch and yaw:
				// Eigen::Matrix3f rotation_matrix;
				// //std::cout << "Pitch: " << pitch << "\n";
				// rotation_matrix << std::cos(pitch),  0, -std::sin(pitch),
				// 				   0, 				    1, 0,
				// 				   std::sin(pitch), 0, std::cos(pitch);
		// 
				// Eigen::Vector3f new_normal = rotation_matrix * normal;
		// 
		// 
				// //std::cout << "New normal: " << new_normal[0] << ", " << new_normal[1] << ", " << new_normal[2] << "\n";
				// roll = - std::atan2(new_normal[1], new_normal[2]); //angle of plane normal about new x axis, with 0 at new z
				// //std::cout << "roll: " << roll << "\n";
				// break;

				//Try RPY (Plane to body?)
				// roll = std::atan2(-normal[1], -normal[2]);
				// rotation_matrix << 1, 0,               0,
				// 				   0, std::cos(roll), -std::sin(roll),
				// 				   0, std::sin(roll),  std::cos(roll);

				// Eigen::Vector3f new_normal = rotation_matrix * normal;

				// pitch = -std::atan2(-new_normal[0], -new_normal[2]);

				//Angles from reference to plane
				roll = atan(normal[1] / normal[2]); //TODO: test this!
				Eigen::Matrix3f rotation_matrix;
				rotation_matrix << 1, 0,               0,
								   0, std::cos(roll), std::sin(roll),
								   0, -std::sin(roll),  std::cos(roll);

				//TODO: re-normalize?

				Eigen::Vector3f new_normal = rotation_matrix * normal;

				pitch = atan(new_normal[0] / new_normal[2]);

				yaw = 0;
				break;
			}
			case (PlaneParser::AngleSolutionType::SIMPLE_ANGLES):
			{
				//std::cout << "Finding simple angles\n";
				//std::cout << "Normal: \n" << normal << "\n";
				//std::cout << "Plane eqn: \n" << plane_coefficients << "\n";
				yaw = 0;
				double norm2 = std::sqrt(std::pow(normal[0], 2) + std::pow(normal[1], 2) + std::pow(normal[2], 2));
				//std::cout << "Norm: " << norm2 << "\n";
				// pitch = std::asin(normal[0]);
				// roll = std::asin(- normal[1]);

				//Angles from reference to plane
				roll = atan(normal[1] / normal[2]); //TODO: test this!
				pitch = atan(normal[0] / normal[2]);
	
				//std::cout << "pitch: " << pitch << "\n";
				//std::cout << "roll: " << roll << "\n";
	
				if (std::isinf(pitch) || std::isnan(pitch)) pitch = 0;
				if (std::isinf(roll) || std::isnan(roll)) roll = 0;
				break;
			}
			default:
			{
				//Find quaternions
				// std::cout << "warning: Quaternion soltion not yet implemented.\n";

				//DO: Find quaternion rotation between two vectors, the plane normal and [0, 0, 1], the normal of the
				// reference x-y plane

				//Axis = v2 cross v1
				//Angle = acos(v1 dot v2)
				//quat = qrot(axis, angle) //Ogre: https://forums.ogre3d.org/viewtopic.php?t=45076

				//OR: https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
				//Shortest arc:
				// Quaternion q;
				// vector a = crossproduct(v1, v2);
				// q.xyz = a;
				// q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2);

				Eigen::Vector3f plane_normal(normal[0], normal[1], normal[2]);
				Eigen::Vector3f ref_normal(0, 0, 1);
				// Eigen::Vector4f quat;

				// double dot = plane_normal.dot(ref_normal);
    //     		if (dot < -0.999999)
    //     		{
    //     			//TODO: convert this mess
    //         		// .cross(tmpvec3, xUnitVec3, a);
    //         		// if (vec3.length(tmpvec3) < 0.000001)
    //           //   	vec3.cross(tmpvec3, yUnitVec3, a);
    //         		// vec3.normalize(tmpvec3, tmpvec3);
    //         		// quat.setAxisAngle(out, tmpvec3, Math.PI);
    //     		} else if (dot > 0.999999) 
    //     		{
    //     		    quat[0] = 0;
    //     		    quat[1] = 0;
    //     		    quat[2] = 0;
    //     		    quat[3] = 1;
    //     		} else 
    //     		{
    //     		    Eigen::Vector3f temp = plane_normal.cross(ref_normal);
    //     		    quat[0] = temp[0];
    //     		    quat[1] = temp[1];
    //     		    quat[2] = temp[2];
    //     		    quat[3] = 1 + dot;
    //     		    return quat.normalize(out, out);
    //     		}
				quat = quat.FromTwoVectors(plane_normal, ref_normal);

			}
		}//end switch
	}

	/**
	 * @function 	getCovariance
	 * @brief 		calculates the covariance matrix for the given plane
	 * @param 		plane_parameters - place to save the covariance/variance
	 * @param 		matricies - struct holding the matricies used in the least squares soltion
	 * @return 		void
	 */
	void PlaneParser::getCovariance(PlaneParser::PlaneParameters& plane_parameters, const PlaneParser::LeastSquaresMatricies& matricies)
	{
		plane_parameters.covariance_matrix = Eigen::Matrix3f::Zero();
		Eigen::VectorXf residual = matricies.points_matrix * matricies.plane_coefficients - matricies.sum_vector;
		switch(settings.covariance_type)
		{
			case (pointcloud_utils::PlaneParser::CovarianceType::GOODNESS_OF_FIT_ERROR):
			{
				//std::cout << "getting goodness of fit error covariance\n";
				float plane_fit_relative_error   = residual.norm() / matricies.sum_vector.norm(); //Norm is L2 norm, output is "relative error"
				//https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
				//(L2 norm IS standard deviation without an extra averaging term, and variance is standard deviation squared) -https://winderresearch.com/workshops/171027_standard_deviation/
				// So, plane_fit_relative_error is essentially standard deviation of the data
				//float plane_fit_relative_error   = (points_matrix * plane_coefficients - sum_vector).norm();

				float variance = std::pow(plane_fit_relative_error, 2) * 1000;

				plane_parameters.variance = variance;

				plane_parameters.covariance_matrix(0,0) = variance;
				plane_parameters.covariance_matrix(1,1) = variance;
				plane_parameters.covariance_matrix(2,2) = variance;
				break;
			}
			case (pointcloud_utils::PlaneParser::CovarianceType::GOODNESS_OF_FIT_DISTANCE):
			{
				//std::cout << "getting goodness of fit distance covariance\n";
				
				//To make it more like a distance to plane: https://mathinsight.org/distance_point_plane
				//std::cout << "Residual: \n";
				//std::cout << residual << "\n";
				float plane_fit_relative_error   = (residual.norm() / matricies.plane_coefficients.norm()) / matricies.points_matrix.rows();
				float variance = std::pow(plane_fit_relative_error, 2) * 1000000;

				plane_parameters.variance = variance;

				plane_parameters.covariance_matrix(0,0) = variance;
				plane_parameters.covariance_matrix(1,1) = variance;
				plane_parameters.covariance_matrix(2,2) = variance;
				break;
			}
			case (pointcloud_utils::PlaneParser::CovarianceType::ANALYTICAL):
			{
				std::cout << "Warning: Analytical covariance has not yet been fully implemented due to difficulties linearizing the plane-to-state equations.\n";
				//Get sigma squared:
				float sigma_2 = residual.transpose() * residual;
				//Get X'X: 
				Eigen::Matrix3f multiplyer = (matricies.points_matrix.transpose() * matricies.points_matrix).inverse();
				
				plane_parameters.covariance_matrix = sigma_2 * multiplyer;
					
				break;
			}
			case (pointcloud_utils::PlaneParser::CovarianceType::MONTE_CARLO):
			{
				std::cout << "Warning: MONTE_Carlo covariance has not yet been implemented.\n";
				break;
			}
			default:
			{
				//Should not get here
			}
		}

		//TODO: add covariance terms to the value print-out
		//std::cout << "Plane states covariance: \n";
		//std::cout << plane_parameters.covariance_matrix << "\n";
		//std::cout << plane_parameters.covariance_matrix(0,0) << ", "
		//		  << plane_parameters.covariance_matrix(0,1) << ", "
		//		  << plane_parameters.covariance_matrix(0,2) << ", "
		//		  << plane_parameters.covariance_matrix(1,0) << ", "
		//		  << plane_parameters.covariance_matrix(1,1) << ", "
		//		  << plane_parameters.covariance_matrix(1,2) << ", "
		//		  << plane_parameters.covariance_matrix(2,0) << ", "
		//		  << plane_parameters.covariance_matrix(2,1) << ", "
		//		  << plane_parameters.covariance_matrix(2,2) << ", \n";
	}

} //end namespace pointcloud_utils
