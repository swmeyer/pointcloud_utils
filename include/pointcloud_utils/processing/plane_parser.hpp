/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 30 Sept 2020
 * Brief: class that does a plane fit to the points in the given window, and reports the motion of the fitted plane
 * File: plane_parser.hpp
 */

#ifndef PLANE_PARSER_HPP
#define PLANE_PARSER_HPP

// --------------------------
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <math.h>

#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

//TODO: save max/min points along plane edges

namespace pointcloud_utils
{
	namespace plane_parser_utils
	{

		enum class CovarianceType
		{
			GOODNESS_OF_FIT_ERROR,
			GOODNESS_OF_FIT_DISTANCE,
			ANALYTICAL,
			MONTE_CARLO,
			LEAST_SQUARES,
			UNKNOWN
		};
	
		enum class AngleSolutionType
		{
			ATTITUDE_ANGLES,
			EULER_ANGLES,
			SIMPLE_ANGLES,
			QUATERNIONS,
			UNKNOWN
		};
		
		enum class PlaneFitType
		{
			SIMPLE,
			SVD,
			UNKNOWN
		};

		struct Settings
		{
			
			bool continue_from_last_plane = false; //If true, will calculate the rates using the elapsed time and state change since last frame
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
			
			plane_parser_utils::AngleSolutionType 	angle_solution_type; 	// Determines what equations to use when solving for the planar roll and pitch
			plane_parser_utils::CovarianceType 		covariance_type; 			// Specifies which equations to use for covariance
			plane_parser_utils::PlaneFitType 		plane_fit_type; //Determines which type of plane fit to use. 
		
			int point_skip_num; //number of points to skip each increment when stepping through cloud
			bool invert_cloud_step_through; //if true, start finding ground points from the largest point index rather than 0
			int max_point_count; //stop collecting points when this number is hit


			float consecutive_height_tolerance;
		};

		struct States
		{
			//INDEX
			enum class index
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
			Eigen::MatrixXf covariance_matrix; // state covariance (usually only for roll, pitch)

			//Quaternion:
			float quat_w = 0;
			float quat_x = 0;
			float quat_y = 0;
			float quat_z = 0;
		
		};

		struct PlaneParameters //equation of plane: a/d x + b/d y + c/d z = 1
		{
			float a_d = 0;
			float b_d = 0;
			float c_d = 0; //TODO:remove these, and have the plane parser set d = 1
			float variance;
			float a;
			float b;
			float c;
			float d;

			Eigen::Matrix3f covariance_matrix; // a, b, c
		};

		// enum class PointValueIndex //used to specify which direction we expect this plane to be in
		// {
		// 	X = 0,
		// 	Y,
		// 	Z
		// };

		struct LeastSquaresMatricies
		{
			Eigen::MatrixXf points_matrix; //mx3
			Eigen::Vector3f plane_coefficients; //3 x 1
			Eigen::VectorXf sum_vector; //m x 1
			Eigen::MatrixXf matrix_U; //mxm
			Eigen::MatrixXf matrix_V; // mx3
			Eigen::Matrix3f matrix_E; // 3x3
		};

		PlaneFitType convertPlaneFitType(const std::string& string_in);
		CovarianceType convertCovarianceType(const std::string& string_in);
		AngleSolutionType convertAngleSolutionType(const std::string& string_in);
	}

	template <class T> class PlaneParser
	{
		public:	
			PlaneParser(const plane_parser_utils::Settings& settings);
			~PlaneParser();
	
			void updateSettings(plane_parser_utils::Settings& settings);
	
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
			 * @param 		intensity_min - min intensity value to consider (default 0)
	 		 * @param 		intensity_max - max intensity value to consider (default 256)
	 		 * @return 		void
			 */
			void parsePlane
			(
				const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in, 
				std::vector<T>& cloud, 
				sensor_msgs::msg::PointCloud2& filtered_cloud,
				plane_parser_utils::PlaneParameters& plane_parameters,
				plane_parser_utils::States& plane_states,
				pointcloud_utils::SearchWindow& search_window,
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
			 * @param 		time - rostime in seconds for this cloud
	 		 * @param 		intensity_min - min intensity value to consider (default 0)
	 		 * @param 		intensity_max - max intensity value to consider (default 256)
	 		 * @return 		void
			 */
			void parsePlane
			(
				std::vector<T>& cloud, 
				std::vector<T>& filtered_cloud,
				plane_parser_utils::PlaneParameters& plane_parameters,
				plane_parser_utils::States& plane_states,
				pointcloud_utils::SearchWindow& search_window,
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
			//  * @return 		void
			//  */
			// void parsePlane
			// (
			// 	std::vector<T>& cloud, 
			// 	sensor_msgs::msg::PointCloud2& filtered_cloud,
			// 	plane_parser_utils::PlaneParameters& plane_parameters,
			// 	plane_parser_utils::States& plane_states,
			// 	pointcloud_utils::SearchWindow& search_window,
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
			bool pointIsOnPlane(const T& pt, const plane_parser_utils::PlaneParameters& plane_parameters, const float& tolerance);
		
		private:
			// --------------------------
			plane_parser_utils::Settings settings; // holds the current settings for this converter
			
			plane_parser_utils::States tracked_plane_states;
			
			double last_state_time;
			double this_state_time;
			bool first = true;
			// --------------------------
	
	
			/**
			 * @Function 	filterCloud
			 * @Param 		cloud - point cloud to parse
			 * @Param 		plane_points - place to save parsed cloud
			 * @Param 		search_window - bounds within which to process points
			 * @param 		intensity_min - minimum intensity to accept into plane (default 0)
			 * @param 		intensity_max - maximum intensity to accept into plane (default 256)
			 * @Return 		void
			 * @Brief 		Filteres the given cloud to the given window of points
			 */
			void filterCloud
			(
				std::vector<T>& cloud, 
				std::vector<T>& plane_points, 
				const pointcloud_utils::SearchWindow& search_window,
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
			void fitPlane(const std::vector<T>& cloud, plane_parser_utils::LeastSquaresMatricies& matricies);
	
			/**
			 * @Function 	removeOutliers
			 * @Param 		plane_points - points belonging to the current plane
			 * @Param 		plane_coefficients - parameters of the current plane fit, with variance
			 * @Reutnr 		int - number of outliers removed
			 * @Brief 		Removes outliers that are out of tolerance of the given plane equation
			 */
			int removeOutliers(std::vector<T>& plane_points, const Eigen::Vector3f& plane_coefficients);
		
			/**
			 * @Function 	getPlaneStates
			 * @param 		plane_parameters - coefficients of the fitted plane equation, with covariance
			 * @param 		plane_states - values representing planar position and orientation (to be found)
			 * @Return 		void
			 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
			 */
			void getPlaneStates
			(
				const plane_parser_utils::PlaneParameters& plane_coefficients,
				plane_parser_utils::States& plane_states
			);
	
			/**
			 * @Function 	getPlaneOrientation
	 		 * @Param 		plane_parameters - a, b, c of plane equation with covariance
			 * @param 		orientation_covariance - covariance for the found angles
			 * @param 		roll - roll angle of the plane (to be found, if requested)
			 * @param 		pitch - pitch angle of the plane (to be found, if requested)
			 * @param 		yaw - yaw angle of the plane (to be found, if requested)
			 * @param 		quat - quaternion of the plane orienation (to be found, if requested)
			 * @Return 		void
			 * @Brief 		determines the orientation of the plane described by the given plane equation
			 */
			void getPlaneOrientation(const plane_parser_utils::PlaneParameters plane_parameters, Eigen::Matrix3f& orientation_covariance, double& roll, double& pitch, double& yaw, Eigen::Quaternion<float>& quat);
	
	
			/**
			 * @function 	getPlaneParameterCovariance
			 * @brief 		calculates the covariance matrix for the given plane parameters
			 * @param 		plane_parameters - place to save the covariance/variance
			 * @param 		matricies - struct holding the matricies used in the least squares soltion
			 * @return 		void
			 */
			void getPlaneParameterCovariance(plane_parser_utils::PlaneParameters& plane_parameters, const plane_parser_utils::LeastSquaresMatricies& matricies);

	}; //end class PlaneParser

} //end namespace pointcloud_utils

#endif //end ifndef PLANE_PARSER_HPP
