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
		PlaneParser::SearchWindow& search_window,
		bool continue_from_last_plane
	)
	{
		if (continue_from_last_plane)
		{
			if (first)
			{
				first = false;
				last_state_time = this_state_time;
			}
			this_state_time = cloud_in->header.stamp.toSec();
		}

		// convert to point vector
		pointcloud_utils::convertFromPointCloud2(cloud_in, cloud);

		//plane parsing!
		std::vector<pointcloud_utils::pointstruct> cloud_parsed;
		Eigen::Vector3f plane_coefficients;
		findPlane( cloud, cloud_parsed, search_window, plane_coefficients, plane_states);

		if (plane_coefficients.size() != 3)
		{
			plane_parameters.a_d = 0;
			plane_parameters.b_d = 0;
			plane_parameters.c_d = 0;
		} else
		{
			plane_parameters.a_d = plane_coefficients[0];
			plane_parameters.b_d = plane_coefficients[1];
			plane_parameters.c_d = plane_coefficients[2];
		}

		getPlaneStates( plane_coefficients, plane_states, search_window, continue_from_last_plane);

		//convert back to pointcloud message:
		filtered_cloud.header = cloud_in->header;
		filtered_cloud.fields = cloud_in->fields;
		filtered_cloud.point_step = cloud_in->point_step;
		filtered_cloud.height = 1;
		filtered_cloud.width = cloud_parsed.size();
		filtered_cloud.row_step = filtered_cloud.point_step * cloud_parsed.size();
		
		//std::cout << "Plane Coefficients: " << plane_parameters.a_d << ", so on\n";
		//std::cout << "Plane states: " << plane_states.x << ", so on\n";

		filtered_cloud.data.resize(filtered_cloud.row_step);
		memcpy(&(filtered_cloud.data[0]), &(cloud_parsed[0]), filtered_cloud.row_step);
	}

	/**
	 * @Function 	findPlane
	 * @Param 		cloud - point cloud to parse
	 * @Param 		plane_points - place to save parsed cloud
	 * @Param 		search_window - bounds within which to process points
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @param 		plane_coefficients - coefficients of the fitted plane equation
	 * @param 		plane_states - values representing planar position and orientation
	 * @Return 		void
	 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
	 */
	void PlaneParser::findPlane
	(
		std::vector<pointcloud_utils::pointstruct>& cloud, 
		std::vector<pointcloud_utils::pointstruct>& plane_points, 
		const PlaneParser::SearchWindow& search_window,
		Eigen::Vector3f& plane_coefficients,
		PlaneParser::States& plane_states
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
	
		//for each point in the cloud, push it to the cloud_parsed if it is not filtered out
		for (pointcloud_utils::pointstruct pt : cloud)
		{
			if (!settings.use_point_track_method)
			{
				if (pt.x <= search_window.x_max && pt.x >= search_window.x_min &&
					pt.y <= search_window.y_max && pt.y >= search_window.y_min &&
					pt.z <= search_window.z_max && pt.z >= search_window.z_min)
				{
					plane_points.push_back(pt);
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
			//std::cout << "Found ground points: " << ground_points.size() << "\n";
			
			//Plane fit over the ground point cluster!
			plane_coefficients = fitPlane(plane_points);
			if (settings.iterate_plane_fit)
			{
				int plane_fit_iterations = 1;
				int outliers_removed = removeOutliers(plane_points, plane_coefficients);
				while (plane_fit_iterations < settings.max_iterations && outliers_removed > 0)
				{
					plane_coefficients = fitPlane(plane_points);
					outliers_removed = removeOutliers(plane_points, plane_coefficients);
				}
			}
		} else
		{

		}
	}

	/**
	 * @Function 	fitPlane
	 * @Param 		cloud - points to fit a plane to
	 * @Return 		Eigen::Vector3f - vector of plane coefficients, a/d, b/d, c/d
	 * @Brief 		Fits a plane equation (a/d * x + b/d * y + c/d * z = 1) to the given point cloud
	 * //TODO: what happens when my plane has d = 0?? (intersects the origin) - I guess the coefficients will approach infinity
	 */
	Eigen::Vector3f PlaneParser::fitPlane(const std::vector<pointcloud_utils::pointstruct>& cloud)
	{
		//std::cout << "Fitting plane to " << cloud.size() << " points\n";
		if (!cloud.size())
		{
			std::cout << "Not enough points to fit plane";
			Eigen::Vector3f empty;
			return empty;
		}
		//Plane fit over the ground point cluster!
		// Plane equation: ax + by + cz = d
		// Least squares generic solvable form: Ax = b
		// Plane ls solvable form: [x y z] * [a/d b/d c/d]^T = 1
		
		//Populate points_matrix, the A matrix:
		int i = 0;
		Eigen::MatrixXf points_matrix = Eigen::MatrixXf::Zero(cloud.size(), 3);
		for (pointcloud_utils::pointstruct pt : cloud)
		{
			points_matrix.block<1,3>(i,0) << pt.x, pt.y, pt.z;
			i++;
		}
		//std::cout << "\n Matrix: " << points_matrix << "\n";
		
		//Create plane_coefficeints, the x vector:
		Eigen::Vector3f plane_coefficients = Eigen::Vector3f::Zero();
		
		//Populate sum_vector, the b vector:
		Eigen::VectorXf sum_vector = Eigen::VectorXf::Constant(cloud.size(), 1, 1);
	
		//Solve least squares:
		plane_coefficients = points_matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(sum_vector);
	
		//std::cout << "Found plane: " << plane_coefficients[0] << "x + " << plane_coefficients[1] << "y + " << plane_coefficients[2] << "z = 1\n";
	
		return plane_coefficients;
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
			//Solve for pt.x distance to plane:
			// a/d x + b/d y + c/d z = 1 -> x = (1 - b/d y - c/d z) / (a/d)
			float x_plane = (1 - plane_coefficients[1] * pt.y - plane_coefficients[2] * pt.z) / plane_coefficients[0];
			if (!pointcloud_utils::inTolerance(x_plane, pt.x, settings.outlier_tolerance))
			{
				outlier_count++;
				continue; //Skip point
			}

			//Solve for pt.y distance to plane:
			float y_plane = (1 - plane_coefficients[0] * pt.x - plane_coefficients[2] * pt.z) / plane_coefficients[1];
			if (!pointcloud_utils::inTolerance(y_plane, pt.y, settings.outlier_tolerance))
			{
				outlier_count++;
				continue; //Skip point
			}

			//Solve for pt.z distance to plane:
			float z_plane = (1 - plane_coefficients[0] * pt.x - plane_coefficients[1] * pt.y) / plane_coefficients[2];
			if (!pointcloud_utils::inTolerance(z_plane, pt.z, settings.outlier_tolerance))
			{
				outlier_count++;
				continue; //Skip point
			}


			plane_points.push_back(pt);

		}

		return outlier_count;
	}

	/**
	 * @Function 	getPlaneStates
	 * @param 		plane_coefficients - coefficients of the fitted plane equation
	 * @param 		plane_states - values representing planar position and orientation (to be found)
	 * @Param 		search_window - bounds within which to process points
	 * @param 		continue_from_last_plane - if true, updates tracked states using this plane fit
	 * @Return 		void
	 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
	 */
	void PlaneParser::getPlaneStates
	(
		const Eigen::Vector3f& plane_coefficients,
		PlaneParser::States& plane_states,
		const PlaneParser::SearchWindow& search_window,
		bool continue_from_last_plane
	)
	{

		//Get center of  the plane for one-dimensional translation tracking
		if (!settings.use_point_track_method)
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

			//Get translation at sensor origin!!
			pointcloud_utils::pointstruct origin_pt; //NOTE: this is designed to work best for a nearly horizontal plane
			//origin_pt.z = -(1 - plane_coefficients[0] * -center_x - plane_coefficients[1] * -center_y) / plane_coefficients[2];
			//origin_pt.x = -(1 - plane_coefficients[1] * -center_y - plane_coefficients[2] * origin_pt.z) / plane_coefficients[0];
			//origin_pt.y = -(1 - plane_coefficients[0] * -center_x - plane_coefficients[2] * origin_pt.z) / plane_coefficients[1];
			origin_pt.x = 1 / plane_coefficients[0];
			origin_pt.y = 1 / plane_coefficients[1];
			origin_pt.z = 1 / plane_coefficients[2];

			//if (std::isinf(origin_pt.x)) origin_pt.x = 0;
			//if (std::isinf(origin_pt.y)) origin_pt.y = 0;
			//if (std::isinf(origin_pt.z)) origin_pt.z = 0;
			//TODO: how to do vel tracking if is inf?
		
			//std::cout << "origin pt: " << origin_pt.x << "\n";
			//Get orientation of the plane:
			double roll, pitch, yaw;
			getPlaneOrientation(plane_coefficients, roll, pitch, yaw, -1, 1, -1, 1 );
			double elapsed_time = (this_state_time - last_state_time);
			if (elapsed_time != 0 && continue_from_last_plane)
			{
				//std::cout << "Elapsed time: " << elapsed_time << "\n";
				//Find relative motion of track point:
				plane_states.roll_vel = (roll - plane_states.roll) / elapsed_time;
				plane_states.pitch_vel = (pitch - plane_states.pitch) / elapsed_time;
				plane_states.yaw_vel = (yaw - plane_states.yaw_vel) / elapsed_time;
				//TODO: track vel in all orientations

				//std::cout << "Elapsed time: " << elapsed_time << "\n";
				//Find relative motion of track point:
				//plane_states.x_vel = (track_pt_x.x - tracked_plane_states.x) / elapsed_time;
				//plane_states.y_vel = (track_pt_y.y - tracked_plane_states.y) / elapsed_time;
				//plane_states.z_vel = (track_pt_z.z - tracked_plane_states.z) / elapsed_time;
				
				plane_states.x_vel = (origin_pt.x - tracked_plane_states.x) / elapsed_time;
				plane_states.y_vel = (origin_pt.y - tracked_plane_states.y) / elapsed_time;
				plane_states.z_vel = (origin_pt.z - tracked_plane_states.z) / elapsed_time;
			}


	
			plane_states.roll = roll;
			plane_states.pitch = pitch;
			plane_states.yaw = yaw;
	
			// plane_states.x = track_pt_x.x;
			// plane_states.y = track_pt_y.y;
			// plane_states.z = track_pt_z.z;

			plane_states.x = origin_pt.x;
			plane_states.y = origin_pt.y;
			plane_states.z = origin_pt.z; //Note: This always gives me the center of the plane window to the sensor origin. 
	
		} else
		{
			std::cout << "Warning! Point track method has not yet been implemented\n";

			//find planar orientation
			// double roll, pitch, yaw;
			// getPlaneOrientation(plane_coefficients, roll, pitch, yaw, plane_direction, -1, 1, -1, 1 );

			// //https://math.stackexchange.com/questions/2249307/orientation-of-a-3d-plane-using-three-points
			// 
			// //TODO: solve a least squares over the three points:
			// // A = [p1, p2, p3]^T << column points
			// // An = [1 1 1]^T, solve for n (normal vector)
			// // normalize n
			// //Project n onto each of x, y, z planes:
			// // v = u - n(n^Tu) (x plane, u = [1 0 0]^T), cos(angle) = v

			//TODO: we will need a specific point sent in from the cloud to track translation here

			// if (track_point_found)
			// {
			// 	double elapsed_time = (this_state_time.toSec() - last_state_time.toSec());
			// 	if (elapsed_time != 0 && continue_from_last_plane)
			// 	{
			// 		//std::cout << "Elapsed time: " << elapsed_time << "\n";
			// 		//Find relative motion of track point:
			// 		plane_states.x_vel = (track_pt.x - tracked_plane_states.x) / elapsed_time;
			// 		plane_states.y_vel = (track_pt.y - tracked_plane_states.y) / elapsed_time;
			// 		plane_states.z_vel = (track_pt.z - tracked_plane_states.z) / elapsed_time;
			// 	}
		// 
			// 	plane_states.x = track_pt.x;
			// 	plane_states.y = track_pt.y;
			// 	plane_states.z = track_pt.z;
		// 
			// 	//std::cout << "Track point position: " << track_pt.x << ", " << track_pt.y << ", " << track_pt.z << "\n";
			// }
		}

		if (continue_from_last_plane)
		{
			last_state_time = this_state_time;
			tracked_plane_states = plane_states;
		}
	}

	/**
	 * @Function 	getPlaneOrientation
	 * @Param 		plane_coefficients - vector of plane equation coefficients, a/d, b/d, c/d
	 * @param 		roll - roll angle of the plane (to be found)
	 * @param 		pitch - pitch angle of the plane (to be found)
	 * @param 		yaw - yaw angle of the plane (to be found)
	 * @Return 		void
	 * @Brief 		determines the orientation of the plane described by the given plane equation
	 */
	void PlaneParser::getPlaneOrientation(const Eigen::Vector3f& plane_coefficients, double& roll, double& pitch, double& yaw, const float min_1, const float max_1, const float min_2, const float max_2)
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
		//std::cout << "Getting plane orientation.\n";	
		// double ad = plane_coefficients[0];
		// double bd = plane_coefficients[1];
		// double cd = plane_coefficients[2];
	// 
		// double a, b, c, d; 
// 
		// //std::cout << "Plane coefficients: " << ad << ", " << bd << ", " << cd << "\n";
	// 
		// float x1, x2, x3;
		// float y1, y2, y3;
		// float z1, z2, z3;
		// switch (solve_for)
		// {
		// 	case (pointValueIndex::X):
		// 	{
		// 		//b/d y + c/d z -1 = -a/d x --> x = -(b/d * d/a) y - (c/d * d/a)z + d/a
		// 		
		// 		float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
		// 		float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
		// 		float d_prime = (1/plane_coefficients[0]);
	// 
		// 		y1 = min_1;
		// 		z1 = min_2;
		// 		x1 = b_prime * y1 + c_prime * z1 + d_prime;
		// 	
		// 		y2 = min_1;
		// 		z2 = max_2;
		// 		x2 = b_prime * y2 + c_prime * z2 + d_prime;
		// 	
		// 		y3 = max_1;
		// 		z3 = max_2;
		// 		x3 = b_prime * y3 + c_prime * z3 + d_prime;
	// 
		// 		break;
		// 	}
		// 	case (pointValueIndex::Y):
		// 	{
		// 		//a/d x + c/d z -1 = -b/d y --> y = -(a/d * d/b) y - (c/d * d/b)z + d/b
		// 		float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
		// 		float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
		// 		float d_prime = (1/plane_coefficients[1]);
		// 		
		// 		x1 = min_1;
		// 		z1 = min_2;
		// 		y1 = a_prime * x1 + c_prime * z1 + d_prime;
		// 	
		// 		x2 = min_1;
		// 		z2 = max_2;
		// 		y2 = a_prime * x2 + c_prime * z2 + d_prime;
		// 	
		// 		x3 = max_1;
		// 		z3 = max_2;
		// 		y3 = a_prime * x3 + c_prime * z3 + d_prime;
		// 	
		// 		break;
		// 	}
		// 	case (pointValueIndex::Z):
		// 	{
		// 		//a/d x + b/d y -1 = -c/d z --> z = -(a/d * d/c) y - (b/d * d/c)y + d/c
		// 		float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
		// 		float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
		// 		float d_prime = (1/plane_coefficients[2]);
	// 
		// 		x1 = min_1;
		// 		y1 = min_2;
		// 		z1 = a_prime * x1 + b_prime * y1 + d_prime;
		// 	
		// 		x2 = min_1;
		// 		y2 = max_2;
		// 		z2 = a_prime * x2 + b_prime * y2 + d_prime;
		// 	
		// 		x3 = max_1;
		// 		y3 = max_2;
		// 		z3 = a_prime * x3 + b_prime * y3 + d_prime;
		// 	
		// 		break;
		// 	}
		// 	default:
		// 	{
		// 		//TODO: what do do here? should never happen
		// 	}
		// }
	// 
		// Eigen::Vector3f pt1(x1, y1, z1);
		// Eigen::Vector3f pt2(x2, y2, z2);
		// Eigen::Vector3f pt3(x3, y3, z3);
	// 
		// Eigen::Vector3f v1 = pt1 - pt2;
		// Eigen::Vector3f v2 = pt3 - pt2;
	
		// Eigen::Vector3f normal = v1.cross(v2);


		Eigen::Vector3f normal = plane_coefficients;//todo: validate
		//Normalize the normal vector:
		double norm = std::sqrt(std::pow(normal[0], 2) + std::pow(normal[1], 2) + std::pow(normal[2], 2));
		normal[0] = normal[0]/norm;
		normal[1] = normal[1]/norm;
		normal[2] = normal[2]/norm;
	
		//World Roll, Pitch, Yaw:
		roll = std::atan2(normal[2], normal[1]);  //angle about world x axis, 0 at y axis (horizontal)
		pitch = std::atan2(normal[2], normal[0]); //angle about world y axis, 0 at x axis (horizontal)
		yaw = std::atan2(normal[1], normal[0]);   //angle about world z axis, 0 at x axis (forward)
	}

} //end namespace pointcloud_utilss