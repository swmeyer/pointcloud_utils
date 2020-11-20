/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 19 Nov 2020
 * Brief: publishes a set planar points
 * File: plane_points_generator.cpp
 */

// --------------------------
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <random>

#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/processing/plane_parser.hpp>
// --------------------------

// --------------------------
bool do_roll_sinusoid; 					// if true, move the plane in a rolling sinusoidal motion
bool do_pitch_sinusoid; 				// if true, move the plane in a pitcing sinusoidal motion
float sinusoid_roll_rate;				// [hz] frequency of roll angle change
float sinusoid_pitch_rate; 				// [hz] frequency of pitch angle change
float sinusoid_pitch_phase_shift; 		// pitch phase shift
float sinusoid_roll_phase_shift; 		// roll phase shift
float current_roll;						// [rad] current roll state
float current_pitch;					// [rad] current pitch state


std::vector<pointcloud_utils::pointstruct> base_cloud; //Planar points without noise
ros::Publisher points_pub; 				 // publisher for the generated point cloud
std::string frame_id; 					 // name of transform frame to publish on
ros::Rate* rate;							// rate at which to publish points

std::normal_distribution<float>* rand_norm; //rnormal distribution
std::default_random_engine* generator;		//random number generator

pointcloud_utils::PlaneParser::PlaneParameters plane_params; //Parameters of the plane equation

int point_density; 						// points per square meter
float x_bound;							// +- x bounds in meters
float y_bound;							// +- y bounds in meters
float z_height;							// [m] height of plane origin
float roll;								// [rad] roll value for plane
float pitch;							// [rad] pitch value for plane
float noise;							// artificial noise standard deviation to add
// --------------------------

void generateMessage(sensor_msgs::PointCloud2& msg)
{
	msg.data.clear();

	std::vector<pointcloud_utils::pointstruct> temp_cloud;

	for (pointcloud_utils::pointstruct pt : base_cloud)
	{
		pt.x += (*rand_norm)(*generator);
		pt.y += (*rand_norm)(*generator);
		pt.z += (*rand_norm)(*generator);
		//pt.intensity += (*rand_norm)(*generator);

		temp_cloud.push_back(pt);
	}

	msg.width = temp_cloud.size();
	msg.row_step = msg.point_step * msg.width;

	msg.data.resize(msg.row_step);

	std::memcpy(&(msg.data[0]), &(temp_cloud[0]), msg.row_step);
}

/**
 * @function 	geneartePlane
 * @brief 		generates a perfect plane using the given bounds and parameters
 * @param 		none
 * @return 		void
 */
void generatePlane()
{
	base_cloud.clear();

	plane_params.a = sin(current_pitch);
	plane_params.b = - sin(current_roll);
	plane_params.c = std::sqrt(1 - std::pow(plane_params.a, 2) - std::pow(plane_params.b, 2));
	plane_params.d = plane_params.c * z_height;

	if (fabs(plane_params.a) < 0.001) plane_params.a = 0;
	if (fabs(plane_params.b) < 0.001) plane_params.b = 0;
	if (fabs(plane_params.c) < 0.001) plane_params.c = 0;
	if (fabs(plane_params.d) < 0.001) plane_params.d = 0;

	plane_params.variance = std::pow(noise, 2);

	float density_1D = (int) std::sqrt(point_density);
	if (!density_1D)
	{
		density_1D = 1;
	}

	//std::cout << "Density: " << density_1D << " per meter\n";
	//std::cout << "Plane eqn: " << plane_params.a << ", " << plane_params.b << ", " << plane_params.c << ", " << plane_params.d << "\n";

	pointcloud_utils::pointstruct pt;
	pt.intensity = 100;
	pointcloud_utils::pointstruct pt2;
	pt2.intensity = 100;
	pointcloud_utils::pointstruct pt3;
	pt3.intensity = 100;
	pointcloud_utils::pointstruct pt4;
	pt4.intensity = 100;

	for (int i = 0; i < x_bound; i++)
	{
		pt.x = i - (float) 1/density_1D;;
		pt2.x = -i + (float) 1/density_1D;;
		pt3.x = i - (float) 1/density_1D;;
		pt4.x = -i + (float) 1/density_1D;;

		for (int i2 = 0; i2 < density_1D; i2++)
		{
			pt.x  += (float) 1/density_1D;
			pt2.x -= (float) 1/density_1D; 
			pt3.x += (float) 1/density_1D;
			pt4.x -= (float) 1/density_1D;

			for (int j = 0; j < y_bound; j++)
			{
				pt.y = j - (float) 1/density_1D;;
				pt2.y = -j + (float) 1/density_1D;;
				pt3.y = -j + (float) 1/density_1D;;
				pt4.y = j - (float) 1/density_1D;;
	
				for (int j2 = 0; j2 < density_1D; j2++)
				{
					pt.y += (float) 1/density_1D;
					pt.z = (plane_params.d - plane_params.a * pt.x - plane_params.b * pt.y) / plane_params.c;
	
					pt2.y -= (float) 1/density_1D;
					pt2.z = (plane_params.d - plane_params.a * pt2.x - plane_params.b * pt2.y) / plane_params.c;
	
					pt3.y -= (float) 1/density_1D;
					pt3.z = (plane_params.d - plane_params.a * pt3.x - plane_params.b * pt3.y) / plane_params.c;
	
					pt4.y += (float) 1/density_1D;
					pt4.z = (plane_params.d - plane_params.a * pt4.x - plane_params.b * pt4.y) / plane_params.c;
	
	
					//std::cout << "X,Y: " << pt.x << ", " << pt.y << ", X,Y2: " << pt2.x << ", " << pt2.y << ", X,Y3: " << pt3.x << ", " << pt3.y << ", X,Y4: " << pt4.x << ", " << pt4.y << "\n";
	
					base_cloud.push_back(pt);
					base_cloud.push_back(pt2);
					base_cloud.push_back(pt3);
					base_cloud.push_back(pt4);
				}
			}
		}
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "point_plane_generator");
	ros::NodeHandle n_("~");
	ros::NodeHandle n;

	std::string lidar_topic;
	n_.param<std::string>("lidar_topic", lidar_topic, "/points");
	n_.param<std::string>("frame_id", frame_id, "lidar");
	n_.param<int>("point_density", point_density, 10);
	n_.param<float>("x_bound", x_bound, 10);
	n_.param<float>("y_bound", y_bound, 10);
	n_.param<float>("z_height", z_height, 0);
	n_.param<float>("roll", roll, 0);
	n_.param<float>("pitch", pitch, 0);
	n_.param<float>("noise", noise, 0.1);

	float pub_rate;
	n_.param<float>("pub_rate", pub_rate, 10);

	n_.param<bool>("rolling_sinusoid", do_roll_sinusoid, true);
	n_.param<bool>("pitching_sinusoid", do_pitch_sinusoid, true);
	n_.param<float>("sinusoid_roll_rate", sinusoid_roll_rate, 0);
	n_.param<float>("sinusoid_pitch_rate", sinusoid_pitch_rate, 0);
	n_.param<float>("sinusoid_pitch_phase_shift", sinusoid_pitch_phase_shift, 0);
	n_.param<float>("sinusoid_roll_phase_shift", sinusoid_roll_phase_shift, 0);

	current_roll = roll;
	current_pitch = pitch;

	generatePlane();

	points_pub = n.advertise<sensor_msgs::PointCloud2>(lidar_topic, 1);

	rate = new ros::Rate(pub_rate);
	rand_norm = new std::normal_distribution<float>(0, noise);
	generator = new std::default_random_engine(ros::Time::now().toSec());

	sensor_msgs::PointCloud2 msg;
	msg.header.frame_id = frame_id;
	msg.height = 1;
	msg.point_step = 32;

	sensor_msgs::PointField field;
	field.name = "x";
	field.offset = 0;
	field.datatype = sensor_msgs::PointField::FLOAT32;
	field.count = 1;
	msg.fields.push_back(field);

	field.name = "y";
	field.offset = 4;
	msg.fields.push_back(field);

	field.name = "z";
	field.offset = 8;
	msg.fields.push_back(field);

	field.name = "intensity";
	field.offset = 16;
	msg.fields.push_back(field);

	field.name = "ring";
	field.offset = 20;
	field.datatype = sensor_msgs::PointField::UINT16;
	msg.fields.push_back(field);

	std::cout << "time, roll, pitch,\n";


	while (ros::ok())
	{
		generateMessage(msg);
		if (do_pitch_sinusoid)
		{
			current_pitch = pitch * std::sin(sinusoid_pitch_rate * ros::Time::now().toSec() + sinusoid_pitch_phase_shift);
		}
		if (do_roll_sinusoid)
		{
			current_roll = roll * std::sin(sinusoid_roll_rate * ros::Time::now().toSec() + sinusoid_roll_phase_shift);
		}
		if (do_pitch_sinusoid || do_roll_sinusoid)
		{
			generatePlane();
		}
		std::cout << ros::Time::now() << ", " << current_roll << ", " << current_pitch << ", \n";
		rate->sleep();
		points_pub.publish(msg);
	}

	return(0);
}