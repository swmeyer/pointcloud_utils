/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 16 March 2021
 * Brief: subscribes to imu data and publishes roll, pitch, and yaw angles from the quaternion orientation on separate channels
 * File: imu_to_euler.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float_32.hpp>
// --------------------------

class ImuToEulerNode : public rclcpp:Node
{
	public:
		ImuToEulerNode() :
			Node("imu_to_euler_node")
		{
			//main content

			//Declare parameters
			this->declare_parameter<bool>("ypr", true);
			this->declare_parameter<bool>("rpy", false);
			this->declare_parameter<bool>("body_angles", false);
			this->declare_parameter<bool>("publish_in_degrees", false);
			this->declare_parameter<std::string>("imu_topic", "/imu/data");
		

			//Get parameters

			this->get_parameter<bool>("ypr", ypr);
			this->get_parameter<bool>("rpy", rpy);
			this->get_parameter<bool>("body_angles", body_angles);
		
			this->get_parameter<bool>("publish_in_degrees", publish_in_degrees);
		
			std::string imu_topic;
			this->get_parameter<std::string>("imu_topic", imu_topic);
		

			imu_sub = this->create_subscription(imu_topic, 1, std::bind(&ImuToEulerNode::imuCallback, this, std::placeholders::_1));
		
			roll_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/roll", 1);
			pitch_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/pitch", 1);
			yaw_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/yaw", 1);

		}

		~ImuToEulerNode()
		{

		}

	private:

		//Variables
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
		
		bool publish_in_degrees;
		
		bool ypr;
		bool rpy;
		bool body_angles;
		
		float conversion = 57.2958; //rad to deg


		//Methods

		/**
		 * @Function 	imuCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to imu messages
		 */
		void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg)
		{
			double w = msg->orientation.w;
			double x = msg->orientation.x;
			double y = msg->orientation.y;
			double z = msg->orientation.z;
		
			double yaw;
			double pitch;
			double roll;
		
			if (rpy)
			{
				//Does roll pitch yaw order (from local to global?)
				yaw   = atan2(-(-2 * w * z + 2 * x * y), std::pow(w, 2) + std::pow(x, 2) - std::pow(y, 2) - std::pow(z, 2));
				pitch = asin(2 * w * y + 2 * x * z);
				roll  = atan2(-(-2 * w * x + 2 * y * z), std::pow(w, 2) - std::pow(x, 2) - std::pow(y, 2) + std::pow(z, 2));
			} else if (body_angles)
			{
				//Does angles with reference to the static body angles rather than a rotation sequence
				yaw = atan2(2 * (w * z + x * y), (std::pow(w, 2) + std::pow(x, 2) - std::pow(y, 2) - std::pow(z, 2))); // From YPR
				roll  = atan2(-(-2 * w * x + 2 * y * z), std::pow(w, 2) - std::pow(x, 2) - std::pow(y, 2) + std::pow(z, 2)); // From RPY
				pitch = -atan2(-2 * w * y + 2 * x * z, std::pow(w, 2) + std::pow(x, 2) - std::pow(y, 2) - std::pow(z, 2)); //From YZX
		
			}	else
			{
				//Does yaw pitch roll order (from local to global?)
				yaw = atan2(2 * (w * z + x * y), (std::pow(w, 2) + std::pow(x, 2) - std::pow(y, 2) - std::pow(z, 2)));
				pitch = asin(2 * (w * y - z * x));
				roll = atan2(2 * (w * x + y * z), (std::pow(w, 2) - std::pow(x, 2) - std::pow(y, 2) + std::pow(z, 2)));
			}
		
			std_msgs::msg::Float32 roll_msg;
			std_msgs::msg::Float32 pitch_msg;
			std_msgs::msg::Float32 yaw_msg;
		
			if (publish_in_degrees)
			{
				roll_msg.data = roll * conversion;
				pitch_msg.data = pitch * conversion;
				yaw_msg.data = yaw * conversion;
			} else
			{
				roll_msg.data = roll;
				pitch_msg.data = pitch;
				yaw_msg.data = yaw;
			}
		
			roll_pub->publish(roll_msg);
			pitch_pub->publish(pitch_msg);
			yaw_pub->publish(yaw_msg);
		}
}