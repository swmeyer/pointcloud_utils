/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 8 Sept 2020
 * Brief: Does a plane fit to the points in the given window, and reports the motion of the fitted plane
 * File: plane_parser.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <math.h>

#include <pointcloud_utils/pointcloud_utils.hpp>
// --------------------------

struct
States
{
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

};

bool print_plane_states;
bool use_point_track_method;

std_msgs::Header header;
std::vector<sensor_msgs::PointField> fields;
uint32_t point_step;

ros::Publisher lidar_pub;
ros::Publisher marker_pub;

States plane_states;

ros::Time last_state_time;
ros::Time this_state_time;
bool first = true;

enum pointValueIndex
{
	X = 0,
	Y,
	Z
};

pointValueIndex plane_direction;

double plane_x_min;
double plane_x_max;
double plane_y_min;
double plane_y_max;
double plane_z_min;
double plane_z_max;


/**
 * @Function 	publishPoints
 * @Param 		cloud - point cloud to publish
 * @Return 		void
 * @Brief 		Publishes the given point cloud as a PointCloud2 message
 */
void publishPoints(const std::vector<pointcloud_utils::pointstruct>& cloud)
{
	//std::cout << "Publishing points!\n";
	sensor_msgs::PointCloud2 msg;
	msg.header = header;
	msg.height = 1;
	msg.width = cloud.size();
	msg.fields = fields;

	//std::cout << "Num points to publish: " << cloud.size() << "\n";

	msg.point_step = point_step;
	msg.row_step = point_step * cloud.size();

	msg.data.resize(msg.row_step);
	memcpy(&(msg.data[0]), &(cloud[0]), msg.row_step);

	lidar_pub.publish(msg);
}

/**
 * @Function 	visualizePlane
 * @Param 		plane_coefficients - the a/d, b/d, c/d coefficients of the plane equation
 * @Param 		id - plane ID number
 * @Param 		solve_for - point value to find on plane 
 * @Param 		min_1 - box point 1 value 1
 * @Param 		max_1 - box point 2 value 1
 * @Param 		min_2 - box point 1 value 2
 * @Param 		max_2 - box point 2 value 2 - together these two points define the box diagonal
 * @Return 		void
 * @Brief 		Publishes the given plane as ros visualization lines
 */
void visualizePlane(const Eigen::Vector3f& plane_coefficients, const int id, const pointValueIndex solve_for, 
					const float min_1, const float max_1, const float min_2, const float max_2)
{
	//std::cout << "Visualizing plane!\n";
	//std::cout << "Solving for: " << solve_for << "\n";
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;
	marker.header = header;
	marker.ns = "plane";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.scale.x = 0.01; //line thickness
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	//add endpoint pairs to the marker.points
	geometry_msgs::Point pt1;
	geometry_msgs::Point pt2;
	geometry_msgs::Point pt3;
	geometry_msgs::Point pt4;

	switch (solve_for)
	{
		case (pointValueIndex::X):
		{
			//b/d y + c/d z -1 = -a/d x --> x = -(b/d * d/a) y - (c/d * d/a)z + d/a
			
			float y1 = min_1;
			float z1 = min_2;
			float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
			float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
			float d_prime = (1/plane_coefficients[0]);
			float x1 = b_prime * y1 + c_prime * z1 + d_prime;
			pt1.x = x1;
			pt1.y = y1;
			pt1.z = z1;
		
			float y2 = min_1;
			float z2 = max_2;
			float x2 = b_prime * y2 + c_prime * z2 + d_prime;
			pt2.x = x2;
			pt2.y = y2;
			pt2.z = z2;
		
			float y3 = max_1;
			float z3 = max_2;
			float x3 = b_prime * y3 + c_prime * z3 + d_prime;
			pt3.x = x3;
			pt3.y = y3;
			pt3.z = z3;
		
			float y4 = max_1;
			float z4 = min_2;
			float x4 = b_prime * y4 + c_prime * z4 + d_prime;
			pt4.x = x4;
			pt4.y = y4;
			pt4.z = z4;
			break;
		}
		case (pointValueIndex::Y):
		{
			//a/d x + c/d z -1 = -b/d y --> y = -(a/d * d/b) y - (c/d * d/b)z + d/b
			
			float x1 = min_1;
			float z1 = min_2;
			float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
			float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
			float d_prime = (1/plane_coefficients[1]);
			float y1 = a_prime * x1 + c_prime * z1 + d_prime;
			pt1.x = x1;
			pt1.y = y1;
			pt1.z = z1;
		
			float x2 = min_1;
			float z2 = max_2;
			float y2 = a_prime * x2 + c_prime * z2 + d_prime;
			pt2.x = x2;
			pt2.y = y2;
			pt2.z = z2;
		
			float x3 = max_1;
			float z3 = max_2;
			float y3 = a_prime * x3 + c_prime * z3 + d_prime;
			pt3.x = x3;
			pt3.y = y3;
			pt3.z = z3;
		
			float x4 = max_1;
			float z4 = min_2;
			float y4 = a_prime * x4 + c_prime * z4 + d_prime;
			pt4.x = x4;
			pt4.y = y4;
			pt4.z = z4;
			break;
		}
		case (pointValueIndex::Z):
		{
			//a/d x + b/d y -1 = -c/d z --> z = -(a/d * d/c) y - (b/d * d/c)y + d/c
			//std::cout << "Solving for z at " << min_1 << ", " << min_2 << "\n";
			float x1 = min_1;
			float y1 = min_2;
			float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
			float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
			float d_prime = (1/plane_coefficients[2]);
			float z1 = a_prime * x1 + b_prime * y1 + d_prime;
			pt1.x = x1;
			pt1.y = y1;
			pt1.z = z1;
		
			float x2 = min_1;
			float y2 = max_2;
			float z2 = a_prime * x2 + b_prime * y2 + d_prime;
			pt2.x = x2;
			pt2.y = y2;
			pt2.z = z2;
		
			float x3 = max_1;
			float y3 = max_2;
			float z3 = a_prime * x3 + b_prime * y3 + d_prime;
			pt3.x = x3;
			pt3.y = y3;
			pt3.z = z3;
		
			float x4 = max_1;
			float y4 = min_2;
			float z4 = a_prime * x4 + b_prime * y4 + d_prime;
			pt4.x = x4;
			pt4.y = y4;
			pt4.z = z4;
			break;
		}
		default:
		{
			//TODO: what do do here? should never happen
		}
	}

	//Line 1-2
	marker.points.push_back(pt1);
	marker.points.push_back(pt2);

	//Line 2-3
	marker.points.push_back(pt2);
	marker.points.push_back(pt3);

	//Line 3-4
	marker.points.push_back(pt3);
	marker.points.push_back(pt4);

	//Line 4-1
	marker.points.push_back(pt4);
	marker.points.push_back(pt1);


	markers.markers.push_back(marker);

	//publish the plane
	marker_pub.publish(markers);
}

/**
 * @Function 	fitPlane
 * @Param 		cloud - points to fit a plane to
 * @Return 		Eigen::Vector3f - vector of plane coefficients, a/d, b/d, c/d
 * @Brief 		Fits a plane equation (a/d * x + b/d * y + c/d * z = 1) to the given point cloud
 */
Eigen::Vector3f fitPlane(const std::vector<pointcloud_utils::pointstruct>& cloud)
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

	//TODO: remove outlier points from the plane, and re-fit!

	return plane_coefficients;
}

/**
 * @Function 	greatestCommonDivisor
 * @Param 		double a
 * @Param 		double b
 * @Param 		double c
 * @return 		double gcd; -1 is fault
 * @brief 		Recursive function to return gcd of a and b - https://www.geeksforgeeks.org/program-find-gcd-floating-point-numbers/
 */
double greatestCommonDivisor(double a, double b) 
{ 

  	//std::cout << "GCD: a: " << a << ", b: " << b << "\n";
    if (a < b) 
    {
    	//std::cout << "Flipping\n";
        return greatestCommonDivisor(b, a); 
    }

    //std::cout << "Recursion no flip\n";
  	
    // base case 
    if (fabs(b) < 0.001) 
    {
    	//std::cout << "Return a\n";
        return a;  
    }
    else
    {
    	if(b != 0)
    	{
    		double new_a = a - floor(a / b) * b;
    		if (floor(a/b) == 0)
    		{
    			return (std::min(a, b));
    		}
    		//std::cout << "Old a: " << a << ", new a: " << new_a << "\n";
        	return (greatestCommonDivisor(b, new_a)); 
    	} else
    	{
    		return -1; 
    	}
    }
}

int gcd(int a, int b) {
   if (b == 0)
   return a;
   return gcd(b, a % b);
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
void getPlaneOrientation(const Eigen::Vector3f& plane_coefficients, double& roll, double& pitch, double& yaw, 
	const pointValueIndex solve_for, const float min_1, const float max_1, const float min_2, const float max_2)
{
	//std::cout << "Getting plane orientation.\n";
	//int ad = plane_coefficients[0] * 10000000;
	//int bd = plane_coefficients[1] * 10000000;
	//int cd = plane_coefficients[2] * 10000000;

	double ad = plane_coefficients[0];
	double bd = plane_coefficients[1];
	double cd = plane_coefficients[2];

	double a, b, c, d; 
	//a = ad / 10000000;
	//b = bd / 10000000;
	//c = cd / 10000000;
	//d = 1;

	//std::cout << "Plane coefficients: " << ad << ", " << bd << ", " << cd << "\n";
//
	////std::cout << "about to get gcd\n";
	//double gcd_a_b = gcd(ad, bd);
	//d = gcd(cd, gcd_a_b);
	////d = gcd(cd, gcd_a_b) / 10000000;
	//if (d == -1)
	//{
	//	std::cout << "ERROR: Failed to find GCD\n";
	//	return;
	//} else if (d != 0)
	//{
	//	a = (ad / d) / 10000000;
	//	b = (bd / d) / 10000000;
	//	c = (cd / d) / 10000000;
	//} else
	//{
	//	std::cout << "ERROR!! GCD is 0 :(\n";
	//}

	//https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
	//Assuming that z is up, x is fwd, y is left
	//std::cout << "about to do angle stuff\n";
	//std::cout << "New coefficeints: " << a << ", " << b << ", " << c << ", " << d << "\n";
	//double magnitude = std::sqrt((ad*ad) + (bd*bd) + (cd*cd));
	//a = ad / magnitude;
	//b = bd / magnitude;
	//c = cd / magnitude;
	////TODO: plot this line and see if it looks like a normal
	//pitch = std::asin(-b);
	//roll = std::atan2(a, c);

	//TODO: get the normal from the cross product of two vectors on the plane:
	//normal = cross((p1 - p2), (p2 - p3);
	//TODO: does this work ok with all planes?
	float x1, x2, x3;
	float y1, y2, y3;
	float z1, z2, z3;
	switch (solve_for)
	{
		case (pointValueIndex::X):
		{
			//b/d y + c/d z -1 = -a/d x --> x = -(b/d * d/a) y - (c/d * d/a)z + d/a
			
			float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
			float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
			float d_prime = (1/plane_coefficients[0]);

			y1 = min_1;
			z1 = min_2;
			x1 = b_prime * y1 + c_prime * z1 + d_prime;
		
			y2 = min_1;
			z2 = max_2;
			x2 = b_prime * y2 + c_prime * z2 + d_prime;
		
			y3 = max_1;
			z3 = max_2;
			x3 = b_prime * y3 + c_prime * z3 + d_prime;

			break;
		}
		case (pointValueIndex::Y):
		{
			//a/d x + c/d z -1 = -b/d y --> y = -(a/d * d/b) y - (c/d * d/b)z + d/b
			float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
			float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
			float d_prime = (1/plane_coefficients[1]);
			
			x1 = min_1;
			z1 = min_2;
			y1 = a_prime * x1 + c_prime * z1 + d_prime;
		
			x2 = min_1;
			z2 = max_2;
			y2 = a_prime * x2 + c_prime * z2 + d_prime;
		
			x3 = max_1;
			z3 = max_2;
			y3 = a_prime * x3 + c_prime * z3 + d_prime;
		
			break;
		}
		case (pointValueIndex::Z):
		{
			//a/d x + b/d y -1 = -c/d z --> z = -(a/d * d/c) y - (b/d * d/c)y + d/c
			float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
			float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
			float d_prime = (1/plane_coefficients[2]);

			x1 = min_1;
			y1 = min_2;
			z1 = a_prime * x1 + b_prime * y1 + d_prime;
		
			x2 = min_1;
			y2 = max_2;
			z2 = a_prime * x2 + b_prime * y2 + d_prime;
		
			x3 = max_1;
			y3 = max_2;
			z3 = a_prime * x3 + b_prime * y3 + d_prime;
		
			break;
		}
		default:
		{
			//TODO: what do do here? should never happen
		}
	}

	Eigen::Vector3f pt1(x1, y1, z1);
	Eigen::Vector3f pt2(x2, y2, z2);
	Eigen::Vector3f pt3(x3, y3, z3);

	Eigen::Vector3f v1 = pt1 - pt2;
	Eigen::Vector3f v2 = pt3 - pt2;

	Eigen::Vector3f normal = v1.cross(v2);

	//Normalize the normal vector:
	double norm = std::sqrt(std::pow(normal[0], 2) + std::pow(normal[1], 2) + std::pow(normal[2], 2));
	normal[0] = normal[0]/norm;
	normal[1] = normal[1]/norm;
	normal[2] = normal[2]/norm;

	//World Roll, Pitch, Yaw:

	//Try 1:
	roll = std::atan2(normal[2], normal[1]);  //angle about world x axis, 0 at y axis (horizontal)
	pitch = std::atan2(normal[2], normal[0]); //angle about world y axis, 0 at x axis (horizontal)
	yaw = std::atan2(normal[1], normal[0]);   //angle about world z axis, 0 at x axis (forward)

	//Try 2: //TODO: make this axis-dependent
	//roll = std::asin(normal[1]);
	//pitch = std::atan2(normal[2], normal[0]);
	//yaw = 0;


	//TODO: get RPY relative to an initial planar orientation, rather than to global (so we get plane-relative angles, which should be cleaner)?

	// //Object Roll, Pitch, Yaw, assuming object x, y, z are aligned with the world
	// // 			(This interprests motion of side, front, and back planes to get object motion)
	// //TODO: validate the equations below:
	// switch(solve_for)
	// {
	// 	case (pointValueIndex::X):
	// 	{
	// 		//X is roughly plane "up", so y and z components will be small or zero (assume y points in old x, z points in old y to keep signs the same)
	// 		//X
	// 		roll = roll;
	// 		pitch = 
// 
// 
	// 		// old: //Term redefine: roll is about y axis, pitch is about z axis, yaw is about x axis
	// 		//roll = pitch;
	// 		//pitch = yaw;
	// 		//yaw = roll;
	// 		//pitch = std::asin(-normal[1]);
	// 		//roll = std::atan2(normal[0], normal[2]);
	// 		break;
	// 	} 
	// 	case (pointValueIndex::Y):
	// 	{
	// 		//Y is roughly up, so x and z components will be small or zero (assume z points in old x, x points in old y to keep signs the same)
	// 		//Term redefine: roll is about z axis, pitch is about x axis, yaw is about y axis
	// 		roll = yaw;
	// 		pitch = roll;
	// 		yaw = pitch;
	// 		//pitch = std::asin(-normal[2]);
	// 		//roll = std::atan2(normal[1], normal[0]);
	// 		break;
	// 	}
	// 	case (pointValueIndex::Z):
	// 	{
	// 		//Zis roughly up, so x and y components will be small or zero (assume same as world orientation)
	// 		//NO CHANGE IN ROLL, PITCH, YAW
	// 		//pitch = std::asin(-normal[0]);
	// 		//roll = std::atan2(normal[2], normal[1]);
	// 		break;
	// 	}
	// 	default:
	// 	{
	// 		//todo: should never get here
	// 	}
	// }
}

/**
 * @Function 	findPlane
 * @Param 		cloud - point cloud to parse
 * @Param 		cloud_parsed - place to save parsed cloud
 * @Return 		void
 * @Brief 		Parses the given cloud for the given window of points and saves the found planar states
 */
void findPlane(std::vector<pointcloud_utils::pointstruct>& cloud, std::vector<pointcloud_utils::pointstruct>& cloud_parsed)
{
	//Find points of interest

	//std::cout << "Finding plane\n";

	float x_max = plane_x_max;
	float x_min = plane_x_min;
	float y_max = plane_y_max;
	float y_min = plane_y_min;
	float z_max = plane_z_max;
	float z_min = plane_z_min;

	//Bounds 2: point track limits
	float x_max2 = 1;
	float x_min2 = -1;
	float y_max2 = 0;
	float y_min2 = -1;
	float z_max2 = 1;
	float z_min2 = -1;

	int intensity_min = 40;
	int intensity_max = 255;

	float x_max_found; //holds the actual min and max of the found cluster
	float x_min_found; //holds the actual min and max of the found cluster
	float y_max_found; //holds the actual min and max of the found cluster
	float y_min_found; //holds the actual min and max of the found cluster
	float z_max_found; //holds the actual min and max of the found cluster
	float z_min_found; //holds the actual min and max of the found cluster

	bool pt_1_found = false;
	bool pt_2_found = false;
	bool pt_3_found = false;

	bool track_point_found = false;

	std::vector<pointcloud_utils::pointstruct> plane_points;

	pointcloud_utils::pointstruct pt1;
	pointcloud_utils::pointstruct pt2;
	pointcloud_utils::pointstruct pt3;

	pointcloud_utils::pointstruct track_pt;

	pointcloud_utils::pointstruct track_pt_x;
	pointcloud_utils::pointstruct track_pt_y;
	pointcloud_utils::pointstruct track_pt_z;

	bool first_pt = true;
	//std::cout << "bounds: " << x_max << ", " << x_min << ", " << y_max << ", " << y_min
	//						<< ", " << z_max << ", " << z_min << "\n";

	//for each point in the cloud, push it to the cloud_parsed if it is not filtered out
	for (pointcloud_utils::pointstruct pt : cloud)
	{
		if (!use_point_track_method)
		{
			if (pt.x <= x_max && pt.x >= x_min &&
				pt.y <= y_max && pt.y >= y_min &&
				pt.z <= z_max && pt.z >= z_min)
			{
				plane_points.push_back(pt);
				cloud_parsed.push_back(pt);
	
				if (first_pt)
				{
					x_max_found = pt.x;
					x_min_found = pt.x;
					y_max_found = pt.y;
					y_min_found = pt.y;
					z_max_found = pt.z;
					z_min_found = pt.z;
					first_pt = false;
				} else
				{
					if (x_max_found < pt.x) x_max_found = pt.x;
					if (x_min_found > pt.x) x_min_found = pt.x;
					if (y_max_found < pt.x) x_max_found = pt.y;
					if (y_min_found > pt.x) x_min_found = pt.y;
					if (z_max_found < pt.x) x_max_found = pt.z;
					if (z_min_found > pt.x) x_min_found = pt.z;
				}
			}
		}

		if (pt.x <= x_max2 && pt.x >= x_min2 &&
			pt.y <= y_max2 && pt.y >= y_min2 &&
			pt.z <= z_max2 && pt.z >= z_min2)
		{
			//cloud_parsed.push_back(pt);
			if (pt.intensity <= intensity_max && 
				pt.intensity >= intensity_min &&
				!track_point_found &&
				pt.y <= -1 && pt.y >= -1.1 &&
				pt.z <= 0.1 && pt.z >= 0)
			{
				track_pt = pt;
				cloud_parsed.push_back(pt);
				track_point_found = true;
			}
			
			if (pt.y <= -0.4 && pt.y >= -0.5 &&
				pt.z <= 0.1 && pt.z >= 0 && 
				!pt_1_found)
			{
				pt_1_found = true;
				pt1 = pt;
				cloud_parsed.push_back(pt);
				//std::cout << "1 found\n";
			}

			if (pt.y <= -0.4 && pt.y >= -0.5 &&
				pt.z <= 0 && pt.z >= -0.1 && 
				!pt_2_found)
			{
				pt_2_found = true;
				pt2 = pt;
				cloud_parsed.push_back(pt);
				//std::cout << "2 found\n";
			}

			if (pt.y <= -0.7 && pt.y >= -0.8 &&
				pt.z <= 0 && pt.z >= -0.1 && 
				!pt_3_found)
			{
				pt_3_found = true;
				pt3 = pt;
				cloud_parsed.push_back(pt);
				//std::cout << "3 found\n";
			}
		}
	}

	//std::cout << "Plane points found: " << plane_points.size() << "\n";

	//std::cout << "Found points of interest: " << cloud_parsed.size() << "\n";
	if (!use_point_track_method)
	{
		//std::cout << "Found ground points: " << ground_points.size() << "\n";
		
		//Plane fit over the ground point cluster!
		Eigen::Vector3f plane_coefficients = fitPlane(plane_points);

		float center_x = (x_max + x_min) / 2;
		float center_y = (y_max + y_min) / 2;
		float center_z = (z_max + z_min) / 2;

		// d = ax + by + cz

		switch(plane_direction)
		{
			case(pointValueIndex::X):
			{
				visualizePlane(plane_coefficients, 0, plane_direction, y_min, y_max, z_min, z_max);
				
				float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
				float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
				float d_prime = (1/plane_coefficients[0]);
				
				track_pt.y = center_y;
				track_pt.z = center_z;
				track_pt.x = b_prime * track_pt.y + c_prime * track_pt.z + d_prime;
			
				cloud_parsed.push_back(track_pt);
				track_point_found = true;
				break;
			}
			case(pointValueIndex::Y):
			{
				visualizePlane(plane_coefficients, 0, plane_direction, x_min, x_max, z_min, z_max);
				
				float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
				float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
				float d_prime = (1/plane_coefficients[1]);
				
				track_pt.x = center_x;
				track_pt.z = center_z;
				track_pt.y = a_prime * track_pt.x + c_prime * track_pt.z + d_prime;
			
				
				cloud_parsed.push_back(track_pt);
				track_point_found = true;
				break;
			}
			case(pointValueIndex::Z):
			{
				visualizePlane(plane_coefficients, 0, plane_direction, x_min, x_max, y_min, y_max);
				
				float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
				float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
				float d_prime = (1/plane_coefficients[2]);
				track_pt.x = center_x;
				track_pt.y = center_y;
				track_pt.z = a_prime * track_pt.x + b_prime * track_pt.y + d_prime;
			
				cloud_parsed.push_back(track_pt);
				track_point_found = true;

				break;
			}
		}

		if (!use_point_track_method)
		{

			//Get planar translations in all directions:
			float b_prime = -(plane_coefficients[1] * (1/plane_coefficients[0]));
			float c_prime = -(plane_coefficients[2] * (1/plane_coefficients[0]));
			float d_prime = (1/plane_coefficients[0]);
			
			track_pt_x.y = center_y;
			track_pt_x.z = center_z;
			track_pt_x.x = b_prime * track_pt.y + c_prime * track_pt.z + d_prime;
	
			float a_prime = -(plane_coefficients[0] * (1/plane_coefficients[1]));
			c_prime = -(plane_coefficients[2] * (1/plane_coefficients[1]));
			d_prime = (1/plane_coefficients[1]);
			
			track_pt_y.x = center_x;
			track_pt_y.z = center_z;
			track_pt_y.y = a_prime * track_pt.x + c_prime * track_pt.z + d_prime;
			
			a_prime = -(plane_coefficients[0] * (1/plane_coefficients[2]));
			b_prime = -(plane_coefficients[1] * (1/plane_coefficients[2]));
			d_prime = (1/plane_coefficients[2]);
			track_pt_z.x = center_x;
			track_pt_z.y = center_y;
			track_pt_z.z = a_prime * track_pt.x + b_prime * track_pt.y + d_prime;
		}
			
		
		//Report plane position at center:
		//float center_x = (x_max_found + x_min_found) / 2;
		//float center_y = (y_max_found + y_min_found) / 2;
		//float center_z = (z_max_found + z_max_found) / 2;

		//cloud_parsed.clear();

		//Get orientation of the plane:
		double roll, pitch, yaw;
		getPlaneOrientation(plane_coefficients, roll, pitch, yaw, pointValueIndex::X, -1, 1, -1, 1 );
		double elapsed_time = (this_state_time.toSec() - last_state_time.toSec());
		if (elapsed_time != 0)
		{
			//std::cout << "Elapsed time: " << elapsed_time << "\n";
			//Find relative motion of track point:
			plane_states.roll_vel = (roll - plane_states.roll) / elapsed_time;
		}

		plane_states.roll = roll;
		plane_states.pitch = pitch;
		plane_states.yaw = yaw;

	} else
	{
		//find planar orientation
		//https://math.stackexchange.com/questions/2249307/orientation-of-a-3d-plane-using-three-points
	
		//TODO: solve a least squares over the three points:
		// A = [p1, p2, p3]^T << column points
		// An = [1 1 1]^T, solve for n (normal vector)
		// normalize n
		//Project n onto each of x, y, z planes:
		// v = u - n(n^Tu) (x plane, u = [1 0 0]^T), cos(angle) = v
	}

	if (!use_point_track_method)
	{
		double elapsed_time = (this_state_time.toSec() - last_state_time.toSec());
		if (elapsed_time != 0)
		{
			//std::cout << "Elapsed time: " << elapsed_time << "\n";
			//Find relative motion of track point:
			plane_states.x_vel = (track_pt_x.x - plane_states.x) / elapsed_time;
			plane_states.y_vel = (track_pt_y.y - plane_states.y) / elapsed_time;
			plane_states.z_vel = (track_pt_z.z - plane_states.z) / elapsed_time;
		}

		plane_states.x = track_pt_x.x;
		plane_states.y = track_pt_y.y;
		plane_states.z = track_pt_z.z;
		
	} else if (track_point_found)
	{
		double elapsed_time = (this_state_time.toSec() - last_state_time.toSec());
		if (elapsed_time != 0)
		{
			//std::cout << "Elapsed time: " << elapsed_time << "\n";
			//Find relative motion of track point:
			plane_states.x_vel = (track_pt.x - plane_states.x) / elapsed_time;
			plane_states.y_vel = (track_pt.y - plane_states.y) / elapsed_time;
			plane_states.z_vel = (track_pt.z - plane_states.z) / elapsed_time;
		}

		plane_states.x = track_pt.x;
		plane_states.y = track_pt.y;
		plane_states.z = track_pt.z;

		//std::cout << "Track point position: " << track_pt.x << ", " << track_pt.y << ", " << track_pt.z << "\n";
	}


}

/**
 * @Function 	parseCloud
 * @Param 		cloud - point cloud to parse
 * @Param 		cloud_parsed - place to save parsed cloud
 * @Return 		void
 * @Brief 		Parses the given cloud by filtering the point intensity within a set range
 */
void parseCloud(std::vector<pointcloud_utils::pointstruct>& cloud, std::vector<pointcloud_utils::pointstruct>& cloud_parsed)
{
	//std::cout << "Parsing cloud\n";
	if (first)
	{
		first = false;
		last_state_time = this_state_time;
	}

	cloud_parsed.clear();

	//std::cout << std::to_string(this_state_time.toSec() * 1000000000 + this_state_time.toNSec()) << ", ";
	std::cout << this_state_time << ", ";

	findPlane(cloud, cloud_parsed);

	if (print_plane_states)
	{
		//std::cout << "Cab states: \n";
		std::cout << plane_states.x << ", " << plane_states.y << ", " << plane_states.z << ", ";
		std::cout << plane_states.roll << ", " << plane_states.pitch << ", " << plane_states.yaw << ", ";
		std::cout << plane_states.x_vel << ", " << plane_states.y_vel << ", " << plane_states.z_vel << ", ";
		std::cout << plane_states.roll_vel << ", " << plane_states.pitch_vel << ", " << plane_states.yaw_vel << ", ";
	}
	
	std::cout << "\n";

	last_state_time = this_state_time;
}


/**
 * @Function 	pointCloudCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//std::cout << "Receiving new point cloud!\n";
	this_state_time = msg->header.stamp;
	//this_state_time = ros::Time::now();
	//std::cout << ros::Time::now() << "\n";
	header = msg->header;
	fields = msg->fields;
	point_step = msg->point_step;

	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointcloud_utils::pointstruct> cloud;
	cloud.resize(msg->width);
	std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);

	std::vector<pointcloud_utils::pointstruct> cloud_parsed;
	parseCloud(cloud, cloud_parsed);

	//std::cout << "Points in parsed cloud " << cloud_parsed.size() << "\n";

	//publishPoints(cloud_parsed);


	//publishPoints(cloud);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "distance_publisher");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string lidar_sub_topic, lidar_pub_topic;
	n_.param<std::string>("cloud_topic_in", lidar_sub_topic, "/velodyne_points");
	n_.param<std::string>("cloud_topic_out", lidar_pub_topic, "/filtered_points");

	std::string marker_pub_topic;
	n_.param<std::string>("visualization_topic", marker_pub_topic, "/markers");

	n_.param<bool>("print_plane_states", print_plane_states, false);

	n_.param<bool>("use_point_track_method", use_point_track_method, false);

	n_.param<double>("plane_x_min", plane_x_min, -10);
	n_.param<double>("plane_x_max", plane_x_max, 10);
	n_.param<double>("plane_y_min", plane_y_min, -10);
	n_.param<double>("plane_y_max", plane_y_max, 10);
	n_.param<double>("plane_z_min", plane_z_min, -10);
	n_.param<double>("plane_z_max", plane_z_max, 10);

	std::string plane_direction_string;
	n_.param<std::string>("plane_direction", plane_direction_string, "x");

	if (plane_direction_string == "X" || plane_direction_string == "x")
	{
		plane_direction = pointValueIndex::X;
	} else if (plane_direction_string == "Y" || plane_direction_string == "y")
	{
		plane_direction = pointValueIndex::Y;
	} else if (plane_direction_string == "Z" || plane_direction_string == "z")
	{
		plane_direction = pointValueIndex::Z;
	}

	// n_.param<int>("intensity_min", intensity_min, 20);
	// n_.param<int>("intensity_max", intensity_max, 250);

	std::cout << "time, ";
	if (print_plane_states)
	{
		std::cout << "plane_x, plane_y, plane_z, plane_roll, plane_pitch, plane_yaw, plane_x_vel, plane_y_vel, plane_z_vel, plane_roll_vel, plane_pitch_vel, plane_yaw_vel, ";
	}
	
	std::cout << "\n";

	lidar_pub = n.advertise<sensor_msgs::PointCloud2>(lidar_pub_topic, 1);
	marker_pub = n.advertise<visualization_msgs::MarkerArray>(marker_pub_topic, 1);
	
	ros::Subscriber lidar_sub = n.subscribe(lidar_sub_topic, 1, &pointCloudCallback);

	ros::spin();

	return 0;
}