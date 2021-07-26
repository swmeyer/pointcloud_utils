/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 30 Sept 2020
 * Brief: class that does a plane fit to the points in the given window, and reports the motion of the fitted plane
 * File: plane_parser.hpp
 */

#ifndef PLANE_PARSER_HPP
#define PLANE_PARSER_HPP

// --------------------------
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

//TODO: save max/min points along plane edges

namespace pointcloud_utils
{
	class PlaneParser
	{
	public:
		enum CovarianceType
		{
			GOODNESS_OF_FIT_ERROR,
			GOODNESS_OF_FIT_DISTANCE,
			ANALYTICAL,
			MONTE_CARLO,
			LEAST_SQUARES
		};

		enum AngleSolutionType
		{
			ATTITUDE_ANGLES,
			EULER_ANGLES,
			SIMPLE_ANGLES,
			QUATERNIONS
		};
		
		enum PlaneFitType
		{
			SIMPLE,
			SVD,
			UNKNOWN
		}


		struct Settings
		{
			bool iterate_plane_fit = false; //If true, will remove outliers to the fit plane and re-fit. 
			//Will iterate max_iterations times, or until the plane fit has no outliers
			int max_iterations; //if iterate_plane_fit is true, will limit the iterations to this number
			float outlier_tolerance; // [m] distance of point to fitted plane to be considered an outlier

			bool use_point_track_method = false; //if true, uses selected individual points on the plane to do plane fit 
										 // and translation tracking. NOTE: tracking individual points is highly subject to noise 
										 // without frame-to-frame filtering. NOTE: this is not yet implemented
		
			int min_points_to_fit; 	//any fewer poiints, and we won't try to do a plane fit
    		bool report_offsets_at_origin; // if true, will give distance from origin to plane in each direction. Otherwise, will report distance vector normal to plane to origin
			
			bool do_transform = false; // if true, transform the cloud before processing
			pointcloud_utils::Transform transform; // transform values to use (Roll Pitch Yaw ordered rotation matrix will be generated)
			std::string transform_frame; //id of cloud after transform
			
			PlaneParser::AngleSolutionType 	angle_solution_type; 	// Determines what equations to use when solving for the planar roll and pitch
			PlaneParser::CovarianceType 	 	covariance_type; 			// Specifies which equations to use for covariance
			PlaneParser::PlaneFitType 	plane_fit_type; //Determines which type of plane fit to use. 
		};

		struct States
		{
			//INDEX
			enum index
			{
				X = 0,
				Y,
				Z,
				ROLL,
				PITCH,
				YAW,
				XVEL,
				YVEL,
				ZVEL,
				ROLL_VEL,
				PITCH_VEL,
				YAW_VEL
			};

			float x 	= 0;
			float y 	= 0;
			float z 	= 0;
			float roll 	= 0;
			float pitch = 0;
			float yaw 	= 0;
		
			float x_vel 	= 0;
			float y_vel 	= 0;
			float z_vel 	= 0;
			float roll_vel 	= 0;
			float pitch_vel = 0;
			float yaw_vel 	= 0;

			float variance = 0.0;
			Eigen::MatrixXf covariance_matrix; //12x12, full state covariance

			//Quaternion:
			float quat_w = 0;
			float quat_x = 0;
			float quat_y = 0;
			float quat_z = 0;
		
		};

		struct PlaneParameters //equation of plane: a/d x + b/d y + c/d z = 1
		{
			float a_d;
			float b_d;
			float c_d; //TODO:remove these, and have the plane parser set d = 1
			float variance;
			float a;
			float b;
			float c;
			float d;

			Eigen::Matrix3f covariance_matrix; // a, b, c
		};

		enum pointValueIndex //used to specify which direction we expect this plane to be in
		{
			X = 0,
			Y,
			Z
		};

		struct LeastSquaresMatricies
		{
			Eigen::MatrixXf points_matrix;
			Eigen::Vector3f plane_coefficients;
			Eigen::VectorXf sum_vector;
		};

		PlaneParser(const PlaneParser::Settings& settings);
		~PlaneParser();

		void updateSettings(PlaneParser::Settings& settings);

		//TODO: make a templated version which requires the specification of a pointstruct type
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
		 * @param 		intensity_min - minimum intensity to accept into plane (default 0)
		 * @param 		intensity_max - maximum intensity to accept into plane (default 256)
		 * @return 		void
		 */
		void parsePlane
		(
			const sensor_msgs::PointCloud2::ConstPtr& cloud_in, 
			std::vector<pointcloud_utils::pointstruct>& cloud, 
			sensor_msgs::PointCloud2& filtered_cloud,
			PlaneParser::PlaneParameters& plane_parameters,
			PlaneParser::States& plane_states,
			pointcloud_utils::SearchWindow& search_window,
			bool continue_from_last_plane,
			const float intensity_min = 0,
			const float intentisty_max = 256
		);

		/**
		 * @function 	parsePlane
		 * @brief 		finds the plane-fit of the given cloud, using the given window bounds
		 * @param 		cloud - inputted 3D point cloud
		 * @param 		plane_points  - space to store the filtered plane points
		 * @param 		plane_parameters - coefficients of the fitted plane equation
		 * @param 		plane_states - values representing planar position and orientation
		 * @param 		search_window - window within which to process points for this plane
		 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
		 * @param 		time - rostime in seconds for this cloud
	 	 * @param 		intensity_min - minimum intensity to accept into plane (default 0)
		 * @param 		intensity_max - maximum intensity to accept into plane (default 256)
		 * @return 		void
		 */
		void parsePlane
		(
			std::vector<pointcloud_utils::pointstruct>& cloud, 
			std::vector<pointcloud_utils::pointstruct>& filtered_cloud,
			PlaneParser::PlaneParameters& plane_parameters,
			PlaneParser::States& plane_states,
			pointcloud_utils::SearchWindow& search_window,
			bool continue_from_last_plane,
			const double time,
			const float intensity_min = 0,
			const float intentisty_max = 256
		);
		
		// //TODO: make a templated version which requires the specification of a pointstruct type
		// /**
		//  * @function 	parsePlane
		//  * @brief 		finds the plane-fit of the given cloud, using the given window bounds
		//  * @param 		cloud - inputted 3D cloud
		//  * @param 		plane_points  - space to store the filtered plane points
		//  * @param 		plane_parameters - coefficients of the fitted plane equation
		//  * @param 		plane_states - values representing planar position and orientation
		//  * @param 		search_window - window within which to process points for this plane
		//  * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
		//  * @return 		void
		//  */
		// void parsePlane
		// (
		// 	std::vector<pointcloud_utils::pointstruct>& cloud, 
		// 	sensor_msgs::PointCloud2& filtered_cloud,
		// 	PlaneParser::PlaneParameters& plane_parameters,
		// 	PlaneParser::States& plane_states,
		// 	pointcloud_utils::SearchWindow& search_window,
		// 	bool continue_from_last_plane
		// );

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
		bool pointIsOnPlane(const pointcloud_utils::pointstruct& pt, const PlaneParser::PlaneParameters& plane_parameters, const float& tolerance);
		
	private:
		// --------------------------
		Settings settings; // holds the current settings for this converter
		
		States tracked_plane_states;
		
		double last_state_time;
		double this_state_time;
		bool first = true;
		// --------------------------

		/**
		 * @Function 	findPlane
		 * @Param 		cloud - point cloud to parse
		 * @Param 		plane_points - place to save parsed cloud
		 * @Param 		search_window - bounds within which to process points
	 	 * @param 		matricies - struct with eigen matricies used in the least squares solution (to be generated by this function)
		 * @param 		plane_states - values representing planar position and orientation
		 * @param 		intensity_min - minimum intensity to accept into plane (default 0)
		 * @param 		intensity_max - maximum intensity to accept into plane (default 256)
		 * @Return 		bool true if successful
		 * 					 false otherwise
		 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
		 * //TODO: iterate the planeFit to isolate the plane from outlier points
		 */
		bool findPlane
		(
			std::vector<pointcloud_utils::pointstruct>& cloud, 
			std::vector<pointcloud_utils::pointstruct>& plane_points, 
			const pointcloud_utils::SearchWindow& search_window,	
			PlaneParser::LeastSquaresMatricies& matricies,
			PlaneParser::States& plane_states,
			const float intensity_min,
			const float intensity_max
		);

		/**
		 * @Function 	fitPlane
		 * @Param 		cloud - points to fit a plane to
		 * @Param 		matricies - saves the matricies used in the least squares soltion 
		 * @Return 		void
 		 * @Brief 		Fits a plane equation (a/d * x + b/d * y + c/d * z = 1) to the given point cloud
		 * //TODO: what happens when my plane has d = 0?? (intersects the origin) - I guess the coefficients will approach infinity
		 * //TODO: look into using robust regression to support outlier tolerance, and brainstorm other techniques for increasing plaraity of underlying points for each fit
		 */
		void fitPlane(const std::vector<pointcloud_utils::pointstruct>& cloud, PlaneParser::LeastSquaresMatricies& matricies);

		/**
		 * @Function 	removeOutliers
		 * @Param 		plane_points - points belonging to the current plane
		 * @Param 		plane_coefficients - parameters of the current plane fit, with variance
		 * @Reutnr 		int - number of outliers removed
		 * @Brief 		Removes outliers that are out of tolerance of the given plane equation
		 */
		int removeOutliers(std::vector<pointcloud_utils::pointstruct>& plane_points, const Eigen::Vector3f& plane_coefficients);
	
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
		void getPlaneStates
		(
			const Eigen::Vector3f& plane_coefficients,
			const Eigen::Matrix3f& plane_parameters_covariance,
			PlaneParser::States& plane_states,
			const pointcloud_utils::SearchWindow& search_window,
			bool continue_from_last_plane
		);

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
		void getPlaneOrientation(const Eigen::Vector3f& plane_coefficients, Eigen::Matrix3f& orientation_covariance, double& roll, double& pitch, double& yaw, Eigen::Quaternion<float>& quat);


		/**
		 * @function 	getCovariance
		 * @brief 		calculates the covariance matrix for the given plane
		 * @param 		plane_parameters - place to save the covariance/variance
		 * @param 		matricies - struct holding the matricies used in the least squares soltion
		 * @return 		void
		 */
		void getCovariance(PlaneParser::PlaneParameters& plane_parameters, const PlaneParser::LeastSquaresMatricies& matricies);

	}; //end class PlaneParser
	
	namespace plane_parser_utils
	{
		pointcloud_utils::PlaneParser::PlaneFitType convertPlaneFitType(const std::string& string_in);
	}

} //end namespace pointcloud_utils

#endif //end ifndef PLANE_PARSER_HPP
