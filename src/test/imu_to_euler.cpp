/** 
 *Author: Stephanie Meyer swm0022@auburn.edu 16 March 2021
 * Brief: subscribes to imu data and publishes roll, pitch, and yaw angles from the quaternion orientation on separate channels
 * File: imu_to_euler.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
// --------------------------

ros::Publisher roll_pub;
ros::Publisher pitch_pub;
ros::Publisher yaw_pub;

bool publish_in_degrees;

bool ypr;
bool rpy;
bool body_angles;

float conversion = 57.2958; //rad to deg

/**
 * @Function 	imuCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to imu messages
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
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

	std_msgs::Float32 roll_msg;
	std_msgs::Float32 pitch_msg;
	std_msgs::Float32 yaw_msg;

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

	roll_pub.publish(roll_msg);
	pitch_pub.publish(pitch_msg);
	yaw_pub.publish(yaw_msg);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "imu_to_euler");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	n_.param<bool>("ypr", ypr, true);
	n_.param<bool>("rpy", rpy, false);
	n_.param<bool>("body_angles", body_angles, false);

	n_.param<bool>("publish_in_degrees", publish_in_degrees, false);

	std::string imu_topic;
	n_.param<std::string>("imu_topic", imu_topic, "/imu/data");

	ros::Subscriber imu_sub = n.subscribe(imu_topic, 1, &imuCallback);

	roll_pub = n.advertise<std_msgs::Float32>("/imu/roll", 1);
	pitch_pub = n.advertise<std_msgs::Float32>("/imu/pitch", 1);
	yaw_pub = n.advertise<std_msgs::Float32>("/imu/yaw", 1);

	ros::spin();
}