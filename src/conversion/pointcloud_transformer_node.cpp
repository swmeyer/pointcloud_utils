/** Author: Stephanie Meyer swmeyer16@gmail.com 16 Nov 2020
 * Brief: Client node for transformer class for point clouds - can handle many different rotation schema
 * File: pointcloud_transformer_node.cpp
 */

// -------------------------------
#include "pointcloud_utils/conversion/pointcloud_transformer.hpp"

#include <ros/ros.h>
// -------------------------------

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "pointcloud_transformer_service");
	ros::NodeHandle n_("~");

	//TODO: get params?
	//TODO: set up this node to advertise a transform service
}