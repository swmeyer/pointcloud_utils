/** Author: Stephanie Meyer swmeyer16@gmail.com 1 July 2020
 * Brief: simple common utilities for pointlcoud operations
 * File: pointcloud_utils.hpp
 */

#ifndef POINTCLOUD_UTILS_HPP
#define POINTCLOUD_UTILS_HPP

// -------------------------------
#include <iostream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <Eigen/Dense>
// -------------------------------

namespace pointcloud_utils
{

	const float PI = 3.1415;

	typedef struct
	{
		double x = 0;
		double y = 0;
		double z = 0;
	} Point;

	// template <class T>
	// struct Cloud
	// {
	// 	std::vector<T> points;
	// 	std::string frame_id;
	// 	double time;
	// 	int num_pts;
	// }

	typedef struct basePointstruct
	{

	} basePointstruct;

	typedef struct
	{
		float x;
		float y;
		float z;
		float dummy;
		float intensity;
		unsigned short int ring;
		unsigned short int dummy2;
		float dummy3;
		float dummy4;

	} pointstruct; //old forestry trucks file velodyne format

	//std::vector<sensor_msgs::msg::PointField> pointstruct_fields =
	//{
	//	{ "x", 0, sensor_msgs::msg::PointField::FLOAT32, 1},
	//	{ "y", 4, sensor_msgs::msg::PointField::FLOAT32, 1},
	//	{ "z", 8, sensor_msgs::msg::PointField::FLOAT32, 1},
	//	{ "intensity", 16, sensor_msgs::msg::PointField::FLOAT32, 1}
	//};
//
	//int pointstruct_step_size = 32; //TODO: does this work for every one?

	typedef struct
	{
		float x;
		float y;
		float z;
	} simplePointstruct;

	typedef struct luminarPointstruct : basePointstruct
	{
		float x;
		float y;
		float z;
		float intensity;
		float ring;
		float time;

	} luminarPointstruct;

	typedef struct luminarPointstruct2 : basePointstruct
	{
		float x;
		float y;
		float z;
		float intensity;
		uint32_t time_us;

	} luminarPointstruct2;


	typedef struct
	{
		float azimuth;
		float range;
		float vertical_angle;
	} sphericalPointstruct;

	typedef struct
	{
		float azimuth;
		float radius;
		float z;
	} polarPointstruct;

	enum class costmapValues
	{
		FREE = 255,
		UNKNOWN = 127,
		OCCUPIED = 0
	};

	struct SearchWindow //defines the bounds within which to process points
	{
		double x_min;
		double x_max;
		double y_min;
		double y_max;
		double z_min;
		double z_max;
	};

	struct Transform
	{
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		double last_angle; //for using true euler rotations, this is the last applied angle rotation (about a repeated axis)
		std::string child_frame;
		std::string parent_frame; //together, child and parent frames are the to and from frames represented by the transform
	};

	struct Parabola //Good for lines and edges!
	{
		double a = 0;
		double b = 0;
		double c = 0;
	};

	struct Cubic //Good for lines and edges!
	{
		double a = 0;
		double b = 0;
		double c = 0;
		double d = 0;
	};

	struct Polynomial
	{
		double a;
		double b;
		double c;
		double d;
		int degree;
		double cov;
	};


	/**
     * @function    findParabolaY
     * @brief       given an x value and parabola parameters, solve for y
     * @param       x - x value to plug in
     * @param       parabola - parabola parameters to use in equation
     * @return      double - y value solved for
     */
    inline double findParabolaY(const double& x, const Parabola& parabola) { return (parabola.a * std::pow(x, 2) + parabola.b * x + parabola.c); }

    /**
     * @function 	isInBounds
     * @brief 		determine if the given point is within the given rectangular bounds
     * @param 		pt - (x,y,z) point to check in bounds
     * @param 		bounds - rectangular bounds to check for point in
     * @return 		true if point is in bounds
     * 					false otherwise
     */
    inline bool isInBounds(const Point& pt, const SearchWindow& bounds) { return ((pt.x <= bounds.x_max) && (pt.x >= bounds.x_min) && (pt.y <= bounds.y_max) && (pt.y >= bounds.y_min) && (pt.y <= bounds.y_max) && (pt.y >= bounds.y_min)); }

	/**
	 * @function getIntensity
	 * @brief    retrieves the intensity of the given point, if it exists
	 * @param    data - point that may have an intensity value
	 * @return 	 float* - pointer to intensity value, if it exists for the given point. Otherwise, will return NULL
	 */
	template <class T> inline const float* getIntensity(const T& data); 
	template<> inline const float* getIntensity<pointstruct>(const pointcloud_utils::pointstruct& data);
	template<> inline const float* getIntensity<luminarPointstruct>(const pointcloud_utils::luminarPointstruct& data);
	
	/**
	 * @function inTolerance
	 * @brief    checks if the two given data are within tolerance of each other
	 * @param    data1 		- first data to compare
	 * @param 	 data2 		- second data to compare
	 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
	 * @return 	 bool - true if data1 and data2 are within tolerance of each other
	 * 				  - false otherwise 
	 */
	inline bool inTolerance(const double data1, const double data2, const double tolerance);

	// //TODO: finish making conversion functions for to- and from- pointstruct vectors and pointcloud2 messages
	// /** 
	//  * @function 	convertToPointCloud2
	//  * @brief 		convert the given point vector into a pointcloud 2 message
	//  * @param 		cloud - pointstruct vector to convert
	//  * @param 		msg - pointcloud2 message to save conversion into
	//  * @return 		void
	//  */
	// template <class T> void convertToPointCloud2(const std::vector<T>& cloud, sensor_msgs::msg::PointCloud2& msg);
	// template<> void convertToPointCloud2<pointsruct>(const std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::msg::PointCloud2& msg);
	
	// /** 
	//  * @function 	convertFromPointCloud2
	//  * @brief 		convert the given pointcloud2 message into a point vector
	//  * @param 		cloud - pointcloud2 message to convert
	//  * @param 		point_vector - pointstruct vector to save conversion into
	//  * @return 		void
	//  */
	// template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, std::vector<T>& point_vector);
	// /** 
	//  * @function 	convertFromPointCloud2
	//  * @brief 		convert the given pointcloud2 message into a point vector
	//  * @param 		cloud - pointcloud2 message to convert
	//  * @param 		point_vector - pointstruct vector to save conversion into
	//  * @return 		void
	//  */
	// template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud, std::vector<T>& point_vector);

	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @param 		max_num_pts - cutoff point in memcpy (-1 for full cloud)
	 * @param 		invert_point_crop - if true, drop points from the front of the data
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, std::vector<T>& point_vector, const int max_num_pts = -1, const bool invert_point_crop = false);
	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @param 		max_num_pts - cutoff point in memcpy (-1 for full cloud)
	 * @param 		invert_point_crop - if true, drop points from the front of the data
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud, std::vector<T>& point_vector, const int max_num_pts = -1, const bool invert_point_crop = false);
	
	
	/**
	 * @function 	transformCloud
	 * @brief 		transforms the given cloud by the given transform (assumes yaw, pitch, roll transform order)
	 * @param 		cloud - points to transform
	 * @param 		transform - transform to use
	 * @param 		invert - if true, invert the given transform before applying. otherwise, use transform directly
	 * @return 		void
	 */
	template <class T> void transformCloud(std::vector<T>& cloud, const Transform& transform, const bool& invert = false);

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_HPP