/** Author: Stephanie Meyer swmeyer16@gmail.com 11 Feb 2020
/* Brief: rough ground height detection, and tracker of error in ground heigh (bounce)
/* File: ground_error_detector.cpp
*/

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
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

double min_ground;
double max_ground;
double avg_ground;
uint   frame_count = 0;
uint   point_count = 1 ;
double avg_error   = 0;

std::string base_frame; // common frame to transform points to
tf2_ros::Buffer* tf_buffer;

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
	geometry_msgs::TransformStamped transform;
	bool transform_found = false;
	  try 
	  {
	  	transform = tf_buffer->lookupTransform(base_frame, input_msg->header.frame_id, ros::Time::now());
      	transform_found = true;
      } catch (tf2::TransformException& ex) 
      {
      	std::cout << "Unable to find transform from " << input_msg->header.frame_id 
      			  << " to " << base_frame << ": " << ex.what() << "\n";
      	return false;
      }
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
	for (pointstruct pt : points)
	{
		if (pt.x < x_max && pt.x > x_min
			&& pt.y < y_max && pt.y > y_min
			&& pt.z < z_max && pt.z > z_min)
		{
			double min_ground = pt.z;
			double max_ground = pt.z;
			double avg_ground = pt.z;

			min_ground = (min_ground > pt.z)? pt.z : min_ground;
			max_ground = (max_ground < pt.z)? pt.z : max_ground;


			avg_error = ( (avg_error * point_count) + (fabs(pt.z - avg_ground)) )/ (point_count + 1);
			avg_ground = (avg_ground * point_count + pt.z) / ++point_count;
		}
		frame_count++;
	}

	std::cout << "ground min: " << min_ground << " max: " << max_ground 
			  << " avg ground: " << avg_ground << " avg error: " << avg_error << "\n"; 
		
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
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ground_error_counter");
	ros::NodeHandle n;
	ros::NodeHandle n_("~");

	tf_buffer = new tf2_ros::Buffer();

	std::string cloud_topic;
	n_.param<std::string>("cloud_topic", cloud_topic, "/cloud");
	n_.param<std::string>("base_frame", base_frame, "base_link");

	n_.param<double>("x_min", x_min, -10);
	n_.param<double>("x_max", x_max, 10);
	n_.param<double>("y_min", y_min, -10);
	n_.param<double>("y_max", y_max, 10);
	n_.param<double>("z_min", z_min, -10);
	n_.param<double>("z_max", z_max, 10);

	ros::Subscriber cloud_sub = n.subscribe(cloud_topic, 1, &pointCloudCallback);

	ros::spin();

	delete(tf_buffer);
	return(0);
}