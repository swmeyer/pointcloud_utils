/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Aug 2020
 * Brief: simple common utilities for pointlcoud operations
 * File: pointcloud_utils_impl.hpp
 */

#ifndef POINTCLOUD_UTILS_IMPL_HPP
#define POINTCLOUD_UTILS_IMPL_HPP

// -------------------------------
#include "pointcloud_utils/pointcloud_utils.hpp"

// -------------------------------

namespace pointcloud_utils
{
	/**
	 * @function getIntensity
	 * @brief    retrieves the intensity of the given point, if it exists
	 * @param    data - point that may have an intensity value
	 * @return 	 float* - pointer to intensity value, if it exists for the given point. Otherwise, will return NULL
	 */
	template <class T> inline const float* getIntensity(const T& data) { return NULL; }  
	template<> inline const float* getIntensity<pointstruct>(const pointcloud_utils::pointstruct& data) { return &data.intensity; }
	template<> inline const float* getIntensity<luminarPointstruct>(const pointcloud_utils::luminarPointstruct& data) { return &data.intensity; }
	/**
	 * @function inTolerance
	 * @brief    checks if the two given data are within tolerance of each other
	 * @param    data1 		- first data to compare
	 * @param 	 data2 		- second data to compare
	 * @param 	 tolerance 	- max magnitude of error to consider in tolerance
	 * @return 	 bool - true if data1 and data2F are within tolerance of each other
	 * 				  - false otherwise 
	 */
	inline bool inTolerance(const double data1, const double data2, const double tolerance) //TODO: this could be an in-line function
	{
		if ( fabs(data1 - data2) <= tolerance )
		{
			return true;
		}
	
		return false;
	}

	// /** 
	//  * @function 	convertToPointCloud2
	//  * @brief 		convert the given point vector into a pointcloud 2 message
	//  * @param 		cloud - pointstruct vector to convert
	//  * @param 		msg - pointcloud2 message to save conversion into
	//  * @return 		void
	//  */
	// template <class T> void convertToPointCloud2(const std::vector<T>& cloud, sensor_msgs::msg::PointCloud2& msg)
	// {
	// 	//Note! Nothing to do in the default class :/
	// 	std::cout << "Unknown point type. Cannot complete conversion.\n";
	// }
	// template<> void convertToPointCloud2<pointsruct>(const std::vector<pointcloud_utils::pointstruct>& cloud, sensor_msgs::msg::PointCloud2& msg)
	// {
	// 	msg.fields = pointcloud_utils::pointstruct_fields;
	// 	msg.point_step = pointcloud_utils::pointstruct_pointstep;
	// 	msg.width = cloud.size();
	// 	msg.height = 1;
	// 	msg.row_step = msg.pointstep * msg.width;
	// 	msg.data.resize(msg.row_step);
	// 	memcpy(&(msg.data[0]), &(cloud[0]), msg.row_step);
	// 
	// }

	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @param 		max_num_pts - cutoff point in memcpy (if -1, use full cloud)
	 * @param 		invert_point_crop - if true, drop points from the front of the data
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, std::vector<T>& point_vector, const int max_num_pts, const bool invert_point_crop)
	{
		//memcpy into a struct (parses the data into the struct values)
		//here, we assume the user knows what they are doing, and print out a statement if it fails
		int num_pts;
		int point_diff = 0;
		if (max_num_pts != -1)
		{
			num_pts = std::min( (int) cloud->width, max_num_pts);
			if ((int) cloud->width > max_num_pts)
			{
				point_diff = abs(max_num_pts - (int) cloud->width);
				// if (point_diff != 0) point_diff--;
			}
		} else
		{
			num_pts = cloud->width;
		}

		try
		{
			point_vector.resize(num_pts);if (invert_point_crop)
			{
				memcpy(&(point_vector[0]), &(cloud->data[point_diff * cloud->point_step]), num_pts * cloud->point_step);
			} else
			{
				memcpy(&(point_vector[0]), &(cloud->data[0]), num_pts * cloud->point_step);
			}
		} catch (...)
		{
			std::cout << "Error: Cannot complete conversion. Is the poinstruct type correct for your cloud?\n";
		}
	}

	/** 
	 * @function 	convertFromPointCloud2
	 * @brief 		convert the given pointcloud2 message into a point vector
	 * @param 		cloud - pointcloud2 message to convert
	 * @param 		point_vector - pointstruct vector to save conversion into
	 * @param 		max_num_pts - cutoff point in memcpy (if -1, use full cloud)
	 * @param 		invert_point_crop - if true, drop points from the front of the data
	 * @return 		void
	 */
	template <class T> inline void convertFromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud, std::vector<T>& point_vector, const int max_num_pts, const bool invert_point_crop)
	{
		//memcpy into a struct (parses the data into the struct values)
		//here, we assume the user knows what they are doing, and print out a statement if it fails
		int num_pts;
		int point_diff = 0;

		if (max_num_pts != -1)
		{
			num_pts = std::min( (int) cloud.width, max_num_pts);
			if ((int) cloud.width > max_num_pts )
			{
				point_diff = abs(max_num_pts - (int) cloud.width);
				// if (point_diff != 0) point_diff--;
			}
		} else
		{
			num_pts = cloud.width;
		}

		// std::cout << "Point diff: " << point_diff << " points: " << cloud.width << "\n";

		try
		{
			point_vector.resize(num_pts);
			if (invert_point_crop)
			{
				memcpy(&(point_vector[0]), &(cloud.data[point_diff * cloud.point_step]), num_pts * cloud.point_step);
			} else
			{
				memcpy(&(point_vector[0]), &(cloud.data[0]), num_pts * cloud.point_step);
			}
		} catch (...)
		{
			std::cout << "Error: Cannot complete conversion. Is the poinstruct type correct for your cloud?\n";
		}
	}
	
	//template<> void convertFromPointCloud2<pointsruct>(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud, std::vector<pointcloud_utils::pointstruct>& point_vector)
	//{
	//	point_vector.resize(cloud->width);
	//	memcpy(&(point_vector[0]), &(cloud->data[0]), cloud.row_step);
	//}

	/**
	 * @function 	transformCloud
	 * @brief 		transforms the given cloud by the given transform (assumes yaw, pitch, roll transform order)
	 * @param 		cloud - points to transform
	 * @param 		transform - transform to use
	 * @return 		void
	 */
	template <class T> void transformCloud(std::vector<T>& cloud, const Transform& transform, const bool& invert)
	{
		//TODO: confirm transform

		double cos_roll  = std::cos(transform.roll);
		double cos_pitch = std::cos(transform.pitch);
		double cos_yaw	 = std::cos(transform.yaw);
		double sin_roll  = std::sin(transform.roll);
		double sin_pitch = std::sin(transform.pitch);
		double sin_yaw	 = std::sin(transform.yaw);

		//generate an affine transform in an eigen matrix from the current plane parameters
		Eigen::Matrix3f rotation_matrix;
		rotation_matrix << cos_yaw * cos_pitch, 	(cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw), 	(sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch),
		                   cos_pitch * sin_yaw, 	(cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll), 	(cos_roll * sin_yaw * sin_pitch - cos_yaw * sin_roll),
		                   -sin_pitch,        		cos_pitch * sin_roll, 									cos_pitch * cos_roll;

		Eigen::Matrix4f transform_matrix;
		transform_matrix << rotation_matrix.row(0), transform.x,
					 rotation_matrix.row(1), transform.y,
					 rotation_matrix.row(2), transform.z,
					 0, 0, 0, 				 1;

		Eigen::MatrixXf point_matrix(4, cloud.size());

		// int count = 0;
		//TODO: a more efficient transform?

		for (uint i = 0; i < cloud.size(); i++) //skip through the cloud by 100's
		{
			T pt = cloud[i];
			point_matrix.col(i) << pt.x, pt.y, pt.z, 1;
		}

		Eigen::MatrixXf transformed_matrix(4, cloud.size());
		if (invert)
		{
			// std::cout << "tf:\n" << transform_matrix.inverse() << "\n\n";
			transformed_matrix = transform_matrix.inverse() * point_matrix;
		} else
		{
			transformed_matrix = transform_matrix * point_matrix;
		}

		for (uint i = 0; i < cloud.size(); i++)
		{
			 cloud[i].x = transformed_matrix.col(i)[0];
			 cloud[i].y = transformed_matrix.col(i)[1];
			 cloud[i].z = transformed_matrix.col(i)[2];
		}
	}

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_UTILS_IMPL_HPP
