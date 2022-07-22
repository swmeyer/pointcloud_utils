/** 
 *Author: Stephanie Meyer swmeyer16@gmail.com 19 Nov 2020
 * Brief: publishes a set planar points
 * File: plane_points_generator.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud_2.hpp>

#include <random>

#include <Eigen/Dense>

#include <pointcloud_utils/pointcloud_utils.hpp>
#include <pointcloud_utils/pointcloud_utils_impl.hpp>
#include <pointcloud_utils/processing/plane_parser.hpp>
// --------------------------

class PlanePointsGeneratorNode : public rclcpp:Node
{
	public:
		PlanePointsGeneratorNode() :
			Node("plane_points_generator_node")
		{
			//main content

			//Declare Parameters
			this->declare_parameter<std::string>("lidar_topic", "/points");
			this->declare_parameter<std::string>("frame_id", "lidar");
			this->declare_parameter<int>("point_density", 10);
			this->declare_parameter<float>("x_bound", 10);
			this->declare_parameter<float>("y_bound", 10);
			this->declare_parameter<float>("z_height", 0);
			this->declare_parameter<float>("roll", 0);
			this->declare_parameter<float>("pitch", 0);
			this->declare_parameter<float>("noise", 0.1);
			this->declare_parameter<float>("yaw_offset", 0);
			this->declare_parameter<float>("pitch_offset", 0);
			this->declare_parameter<float>("roll_offset", 0);
			this->declare_parameter<float>("pub_rate", 10);
			this->declare_parameter<bool>("rolling_sinusoid", true);
			this->declare_parameter<bool>("pitching_sinusoid", true);
			this->declare_parameter<float>("sinusoid_roll_rate", 0);
			this->declare_parameter<float>("sinusoid_pitch_rate", 0);
			this->declare_parameter<float>("sinusoid_pitch_phase_shift", 0);
			this->declare_parameter<float>("sinusoid_roll_phase_shift", 0);
		
			this->declare_parameter<bool>("incremental_angle_test", false);
			this->declare_parameter<float>("roll_increment", 0);
			this->declare_parameter<float>("pitch_increment", 0);
		
		

			//Get Parameters

			std::string lidar_topic;
			this->get_parameter<std::string>("lidar_topic", lidar_topic);
			this->get_parameter<std::string>("frame_id", frame_id);
			this->get_parameter<int>("point_density", point_density);
			this->get_parameter<float>("x_bound", x_bound);
			this->get_parameter<float>("y_bound", y_bound);
			this->get_parameter<float>("z_height", z_height);
			this->get_parameter<float>("roll", roll);
			this->get_parameter<float>("pitch", pitch);
			this->get_parameter<float>("noise", noise);
		
			this->get_parameter<float>("yaw_offset", yaw_offset);
			this->get_parameter<float>("pitch_offset", pitch_offset);
			this->get_parameter<float>("roll_offset", roll_offset);
		
			float pub_rate;
			this->get_parameter<float>("pub_rate", pub_rate);
		
			this->get_parameter<bool>("rolling_sinusoid", do_roll_sinusoid);
			this->get_parameter<bool>("pitching_sinusoid", do_pitch_sinusoid);
			this->get_parameter<float>("sinusoid_roll_rate", sinusoid_roll_rate);
			this->get_parameter<float>("sinusoid_pitch_rate", sinusoid_pitch_rate);
			this->get_parameter<float>("sinusoid_pitch_phase_shift", sinusoid_pitch_phase_shift);
			this->get_parameter<float>("sinusoid_roll_phase_shift", sinusoid_roll_phase_shift);
		
			this->get_parameter<bool>("incremental_angle_test", incremental_angle_test);
			this->get_parameter<float>("roll_increment", roll_increment);
			this->get_parameter<float>("pitch_increment", pitch_increment);
		
		
			points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1);
		
			rate = new rclcpp::Rate(pub_rate);
			rand_norm = new std::normal_distribution<float>(0, noise);
			generator = new std::default_random_engine(rclcpp::Time::now().toSec());
		
			//Prepare message metadata
			sensor_msgs::msg::PointCloud2 msg;
			msg.header.frame_id = frame_id;
			msg.height = 1;
			msg.point_step = 32;
		
			sensor_msgs::msg::PointField field;
			field.name = "x";
			field.offset = 0;
			field.datatype = sensor_msgs::msg::PointField::FLOAT32;
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
			field.datatype = sensor_msgs::msg::PointField::UINT16;
			msg.fields.push_back(field);
		
			std::cout << "time, roll, pitch,\n";
		
			current_roll = roll;
			current_pitch = pitch;
		
			generatePlane();
			current_time = rclcpp::Time::now();
			generateMessage(msg);
			msg.header.stamp = current_time;
		
			std::cout << current_time << ", " << current_roll << ", " << current_pitch << ", \n";
			rate->sleep();
			points_pub->publish(msg);
		
			while (rclcpp::ok())
			{
				if (incremental_angle_test)
				{
					current_roll += roll_increment;
					current_pitch += pitch_increment;
					generatePlane();
				}
		
				//Update the pitch and roll, if motion is occuring
				if (do_pitch_sinusoid)
				{
					current_pitch = pitch * std::sin(sinusoid_pitch_rate * rclcpp::Time::now().toSec() + sinusoid_pitch_phase_shift);
				}
				if (do_roll_sinusoid)
				{
					current_roll = roll * std::sin(sinusoid_roll_rate * rclcpp::Time::now().toSec() + sinusoid_roll_phase_shift);
				}
		
				//Update the generated plane, if the roll or pitch have changed
				if (do_pitch_sinusoid || do_roll_sinusoid)
				{
					generatePlane();
				}
		
				current_time = rclcpp::Time::now();
				generateMessage(msg);
				msg.header.stamp = current_time;
		
				std::cout << current_time << ", " << current_roll << ", " << current_pitch << ", \n";
				rate->sleep();
				points_pub->publish(msg);
			}
		}

		~PlanePointsGeneratorNode()
		{

		}

	private:

		//Variables

		bool do_roll_sinusoid; 					// if true, move the plane in a rolling sinusoidal motion
		bool do_pitch_sinusoid; 				// if true, move the plane in a pitcing sinusoidal motion
		float sinusoid_roll_rate;				// [hz] frequency of roll angle change
		float sinusoid_pitch_rate; 				// [hz] frequency of pitch angle change
		float sinusoid_pitch_phase_shift; 		// pitch phase shift
		float sinusoid_roll_phase_shift; 		// roll phase shift
		float current_roll;						// [rad] current roll state
		float current_pitch;					// [rad] current pitch state
		rclcpp::Time current_time;					// time stamp for current cloud
		
		float yaw_offset;						// [rad] offset of truth axes from LiDAR axes
		float pitch_offset;						// [rad] offset of truth axes from LiDAR axes
		float roll_offset; 						// [rad] offset of truth axes from LiDAR axes
		
		bool incremental_angle_test; 			// if true, increase the static pitch and roll values by 0.5 deg (0.008727 rad) each frame
		float roll_increment; 					// [rad] amount to increase the roll by each frame
		float pitch_increment; 					// [rad] amount to increase the pitch by each frame
		
		
		std::vector<pointcloud_utils::pointstruct> base_cloud; //Planar points without noise
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub; 				 // publisher for the generated point cloud
		std::string frame_id; 					 // name of transform frame to publish on
		rclcpp::Rate* rate;							// rate at which to publish points
		
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


		//Variables

		/**
		 * @function 	rotatePoint
		 * @brief 		Rotates the position of the current point in its reference frame by the given angles
		 * 				using a YPR rotation sequence
		 * @param 		pt - pt to rotate
		 * @param 		yaw - [rad] yaw rotation angle
		 * @param 		pitch - [rad] pitch rotation angle
		 * @param 		roll - [rad] roll rotation angle
		 * @return 		void 
		 */
		void rotatePoint(pointcloud_utils::pointstruct& pt, const float yaw, const float pitch, const float roll)
		{
		
			float rxx = cos(yaw)*cos(pitch); 
			float rxy = cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw);
			float rxz = sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch);
		
			float ryx = cos(pitch)*sin(yaw);
			float ryy = cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll);
			float ryz = cos(roll)*sin(yaw)*sin(pitch) - cos(yaw)*sin(roll);
		
			float rzx = -sin(pitch);
			float rzy = cos(pitch)*sin(roll);
			float rzz = cos(pitch)*cos(roll);
		
			pointcloud_utils::pointstruct orig;
			orig.x = pt.x;
			orig.y = pt.y;
			orig.z = pt.z;
		
			pt.x = rxx * orig.x + rxy * orig.y + rxz * orig.z;
			pt.y = ryx * orig.x + ryy * orig.y + ryz * orig.z;
			pt.z = rzx * orig.x + rzy * orig.y + rzz * orig.z;
		}
		
		/**
		 * @function 	addNoise
		 * @brief 		adds random, gaussian noise to the position data of the given point
		 * @param 		pt - point to add noise to
		 * @return 		void
		 */
		void addNoise(pointcloud_utils::pointstruct& pt)
		{
			pt.x += (*rand_norm)(*generator);
			pt.y += (*rand_norm)(*generator);
			pt.z += (*rand_norm)(*generator);
		}
		
		/**
		 * @function 	rotateCloud
		 * @brief 		Rotates the current base cloud by the given angles in a ypr sequence
		 * @param 		yaw - [rad] yaw angle to rotate by
		 * @param 		pitch - [rad] pitch angle to rotate by
		 * @param 		roll - [rad] roll angle to rotate by
		 * @return 		void
		 */
		void rotateCloud(const float yaw, const float pitch, const float roll)
		{
			//Note: easiest way to rotate whole cloud is using Eigen
		
			//Use the pointcloud utils function!!
			pointcloud_utils::Transform transform;
			transform.x = 0;
			transform.y = 0;
			transform.z = 0;
			transform.roll = roll;
			transform.pitch = pitch;
			transform.yaw = yaw;
		
			pointcloud_utils::transformCloud(base_cloud, transform);
		}
		
		/**
		 * @function 	generateMessage
		 * @brief 		Takes the current base_cloud and saves it in the given message structure
		 * @param 		msg - message structure used to store the generated message
		 * @return 		void
		 */
		void generateMessage(sensor_msgs::msg::PointCloud2& msg)
		{
			msg.data.clear();
		
			//std::vector<pointcloud_utils::pointstruct> temp_cloud;
		
			//for (pointcloud_utils::pointstruct pt : base_cloud)
			//{
			//	// addNoise(pt)
			//	//pt.intensity += (*rand_norm)(*generator);
		//
			//	temp_cloud.push_back(pt);
			//}
		
			msg.width = base_cloud.size();
			msg.row_step = msg.point_step * msg.width;
		
			msg.data.resize(msg.row_step);
		
			std::memcpy(&(msg.data[0]), &(base_cloud[0]), msg.row_step);
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
							
							// //Rotate the points according to the offsets: 
							// rotatePoint(pt, yaw_offset, pitch_offset, roll_offset);
							// rotatePoint(pt2, yaw_offset, pitch_offset, roll_offset);
							// rotatePoint(pt3, yaw_offset, pitch_offset, roll_offset);
							// rotatePoint(pt4, yaw_offset, pitch_offset, roll_offset);
		
							//TODO: make the noise in the radial direction more pronounced
							addNoise(pt);
							addNoise(pt2);
							addNoise(pt3);
							addNoise(pt4);
		
							base_cloud.push_back(pt);
							base_cloud.push_back(pt2);
							base_cloud.push_back(pt3);
							base_cloud.push_back(pt4);
						}
					}
				}
			}
		
			//Rotate the entire cloud
			rotateCloud(yaw_offset, pitch_offset, roll_offset);
		}
}