/** Author: Stephanie Meyer swmeyer16@gmail.com 11 Feb 2020
/* Brief: rough ground height detection, and tracker of error in ground heigh (bounce)
/* File: ground_error_detector.cpp
*/

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
// --------------------------

// --------------------------
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

} pointstruct;

// Area of interest bounds:
double x_min;
double x_max;
double y_min;
double y_max;
double z_min;
double z_max;

// double min_ground;
// double max_ground;
// double avg_ground;
// uint   frame_count = 0;
// uint   point_count = 1 ;
// double avg_error   = 0;
double last_frame_avg = 0;
double last_frame_dev = 0;

ros::Publisher marker_pub;

std::string base_frame; // common frame to transform points to
std::string lidar_frame; // lidar transform frame
geometry_msgs::TransformStamped transform;
bool transform_found = false; //marks whether static transform has been found

std::string frame_time; //holds current frame time

// --------------------------

//========================================
/// @fn         transformToOutputFrame
/// @brief      handle transform to base frame
/// @param      input_msg - untransformed message
/// @param      output_msg - holder for all transformed messages
/// @return     bool - true if the transform is successful, else false
/// @details    Converts the given input to the common output frame
/// @author     Stephanie Meyer
//========================================
bool transformToOutputFrame(
    const sensor_msgs::PointCloud2::ConstPtr& input_msg,
    sensor_msgs::PointCloud2& output_msg
)
{
	tf2::doTransform (*input_msg, output_msg, transform);
	return true;
}

//========================================
// @fn 		groundStats
// @param 	msg incoming data
// @return 	void
// @brief 	finds ground close to sensor source and reports average error of ground
//========================================
void groundStats(std::vector<pointstruct>& points)
{
	//std::cout << "\n";
	uint   frame_point_count = 1;
	double frame_min = 0;
	double frame_max = 0;
	double frame_avg = 0;
	double frame_std_dev; 
	for (pointstruct pt : points)
	{
		//std::cout << "x: " << pt.x << " y: " << pt.y << " z: " << pt.z << "\n";
		if (pt.x < x_max && pt.x > x_min
			&& pt.y < y_max && pt.y > y_min
			&& pt.z < z_max && pt.z > z_min)
		{
			if (!frame_min && !frame_max && !frame_avg)
			{
				frame_min = pt.z;
				frame_max = pt.z;
				frame_avg = pt.z;
			}

			frame_min = (frame_min > pt.z)? pt.z : frame_min;
			frame_max = (frame_max < pt.z)? pt.z : frame_max;

			frame_std_dev = std::sqrt( ( std::pow( (frame_std_dev * frame_point_count), 2) +  std::pow( (pt.z - frame_avg), 2) ) ) / (frame_point_count + 1);
			frame_avg = (frame_avg * frame_point_count + pt.z) / ++frame_point_count;
		}
	}

	//std::cout << "ground min: " << frame_min << " max: " << frame_max
	//		  << " avg ground: " << frame_avg << " avg dev: " << frame_std_dev << "\n";

	if (!last_frame_avg && !last_frame_dev)
	{
		last_frame_avg = frame_avg;
		last_frame_dev = frame_std_dev;
	}

	//std::cout << "Frame-to-frame change: avg: "  << (frame_avg - last_frame_avg) << " dev: " << (frame_std_dev - last_frame_dev) << "\n"; 
	std::cout << (frame_avg - last_frame_avg) << ", ";
	std::cout << (frame_avg) << ", ";
	std::cout << frame_time << "\n";
	//std::cout << "points: " << frame_point_count << "\n";
}

//========================================
// @fn 		pointCloudCallback
// @param 	msg incoming data
// @return 	void
// @brief 	reacts to incoming data messages
//========================================
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	sensor_msgs::PointCloud2 base_cloud;
	transformToOutputFrame(msg, base_cloud);
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointstruct> cloud;
	cloud.resize(msg->width);
	std::memcpy(&(cloud[0]), &(base_cloud.data[0]), msg->row_step);

	frame_time = std::to_string(msg->header.stamp.toNSec());

	groundStats(cloud);
}

//========================================
// @fn 		tfCallback
// @param 	msg incoming tfs
// @return 	void
// @brief 	reacts to incoming tf messages
//========================================
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	std::cout << "\n";
	for (geometry_msgs::TransformStamped tf : msg->transforms)
	{
		if (tf.header.frame_id == base_frame && tf.child_frame_id == lidar_frame)
		{
			transform = tf;
			transform_found = true;
			return;
		}
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ground_error_counter");
	ros::NodeHandle n;
	ros::NodeHandle n_("~");

	std::string cloud_topic;
	n_.param<std::string>("cloud_topic", cloud_topic, "/cloud");
	n_.param<std::string>("base_frame", base_frame, "base_link");
	n_.param<std::string>("lidar_frame", lidar_frame, "lidar_link");

	n_.param<double>("x_min", x_min, -10);
	n_.param<double>("x_max", x_max, 10);
	n_.param<double>("y_min", y_min, -10);
	n_.param<double>("y_max", y_max, 10);
	n_.param<double>("z_min", z_min, -10);
	n_.param<double>("z_max", z_max, 10);

    ros::Subscriber tf_sub = n.subscribe("/tf", 1, &tfCallback);

    while (ros::ok() && !transform_found)
    {
    	ros::Rate(10).sleep();
    	std::cout << "waiting for transform.\n";
    	ros::spinOnce();
    }

    tf_sub.shutdown();

    std::cout << "change_in_avg_height, avg_height, time(nsec)\n";

	ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1, &pointCloudCallback);

	ros::spin();
	return(0);
}