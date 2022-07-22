/** Author: Stephanie Meyer swmeyer16@gmail.com 16 Nov 2020
 * Brief: Client node for transformer class for point clouds - can handle many different rotation schema
 * File: pointcloud_transformer_node.cpp
 */

// -------------------------------
#include "rclcpp/rclcpp.hpp"

#include "pointcloud_utils/conversion/pointcloud_transformer.hpp"
// -------------------------------

class PointCloudTransformerNode : public rclcpp:Node
{
	public:
		PointCloudTransformerNode() :
			Node ("pointcloud_transformer_node")
		{
			//main content

			//TODO: declare/get some params?

			//TODO: set up this node to advertise a transform service
		}

		~PointCloudTransformerNode()
		{

		}

	private:
}