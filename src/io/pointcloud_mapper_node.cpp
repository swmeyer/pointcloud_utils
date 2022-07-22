/** Author: Stephanie Meyer swmeyer16@gmail.com 5 March 2022
 * Brief: ros wrapper for a class to build a single point map out of localization estimates and incoming point clouds
 * File: pointcloud_mapper_node.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud_utils/io/pointcloud_mapper.hpp"
#include "atr_msgs/msg/state_estimate.hpp"
#include "atr_msgs/msg/localization.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
// --------------------------

class PointCloudMapperNode : public rclcpp::Node
{
	public:
		PointCloudMapperNode() : 
			Node("pointcloud_mapper_node")
		{
			//main content

			//Declare params
			this->declare_parameter<std::string>("cloud_topic", "/luminar_front_points");
			this->declare_parameter<std::string>("location_topic", "/location");
			this->declare_parameter<std::string>("raw_location_topic", "/localization/inspva");
			this->declare_parameter<std::string>("gps_topic", "/novatel_bottom/inspva");

			this->declare_parameter<std::string>("pointfilename_base", "points");
			this->declare_parameter<std::string>("locationfilename", "location.csv");
			this->declare_parameter<std::string>("rawlocationfilename", "raw_location.csv");
			this->declare_parameter<std::string>("gpsfilename", "gps.csv");
			this->declare_parameter<std::string>("mapfilename", "IMS_trackmap.csv");

			this->declare_parameter<double>("time_tolerance", 0.1);
			this->declare_parameter<double>("time_offset", 0.0);

			this->declare_parameter<bool>("save_transformed_points", true);

			this->declare_parameter<int>("rate_ms", 1000); //was 200

			this->declare_parameter<bool>("use_raw_location", true);
			this->declare_parameter<bool>("use_gps", false);

			this->declare_parameter<double>("altitude_max", 240); //was 195, or 224
			this->declare_parameter<double>("ref_lat",  39.809786); //IMS default
			this->declare_parameter<double>("ref_long", -86.235148);
			this->declare_parameter<double>("ref_alt",  0.0);

			this->declare_parameter<double>("downsample_percent", 0.2);
			this->declare_parameter<double>("x_max", 200);
			this->declare_parameter<double>("x_min", -200);
			this->declare_parameter<double>("y_max", 30);
			this->declare_parameter<double>("y_min", -30);
			this->declare_parameter<double>("z_max", 10);
			this->declare_parameter<double>("z_min", -10);

			this->declare_parameter<double>("ego_x_max", 5.0);
			this->declare_parameter<double>("ego_x_min", -3.0);
			this->declare_parameter<double>("ego_y_max", 2.0);
			this->declare_parameter<double>("ego_y_min", -2.0);
			this->declare_parameter<double>("ego_z_max", 10.0);
			this->declare_parameter<double>("ego_z_min", -10.0);
		

			//Get params
			std::string lidar_topic, location_topic, raw_location_topic, gps_topic;
			std::string pointfilename_base, locationfilename, rawlocationfilename, mapfilename;
			int rate;
			double ref_lat, ref_long, ref_alt;

			this->get_parameter<std::string>("cloud_topic", lidar_topic);
			this->get_parameter<std::string>("location_topic", location_topic);
			this->get_parameter<std::string>("raw_location_topic", raw_location_topic);
			this->get_parameter<std::string>("gps_topic", gps_topic);

			this->get_parameter<std::string>("pointfilename_base", settings.pointfilename_base);
			this->get_parameter<std::string>("locationfilename", settings.locationfilename);
			this->get_parameter<std::string>("rawlocationfilename", settings.rawlocationfilename);
			this->get_parameter<std::string>("gpsfilename", settings.gpsfilename);
			this->get_parameter<std::string>("mapfilename", settings.mapfilename);

			this->get_parameter<double>("time_tolerance", settings.time_tolerance);
			this->get_parameter<double>("time_offset", settings.time_offset);

			this->get_parameter<bool>("save_transformed_points", settings.save_transformed_points);

			this->get_parameter<int>("rate_ms", rate);

			this->get_parameter<bool>("use_raw_location", settings.use_raw_location);
			this->get_parameter<bool>("use_gps", settings.use_gps);

			this->get_parameter<double>("altitude_max", settings.altitude_max);
			this->get_parameter<double>("ref_lat", ref_lat);
			this->get_parameter<double>("ref_long", ref_long);
			this->get_parameter<double>("ref_alt", ref_alt);

			this->get_parameter<double>("downsample_percent", settings.downsample_percent);
			this->get_parameter<double>("x_max", settings.x_max);
			this->get_parameter<double>("x_min", settings.x_min);
			this->get_parameter<double>("y_max", settings.y_max);
			this->get_parameter<double>("y_min", settings.y_min);
			this->get_parameter<double>("z_max", settings.z_max);
			this->get_parameter<double>("z_min", settings.z_min);

			this->get_parameter<double>("ego_x_max", settings.ego_x_max);
			this->get_parameter<double>("ego_x_min", settings.ego_x_min);
			this->get_parameter<double>("ego_y_max", settings.ego_y_max);
			this->get_parameter<double>("ego_y_min", settings.ego_y_min);
			this->get_parameter<double>("ego_z_max", settings.ego_z_max);
			this->get_parameter<double>("ego_z_min", settings.ego_z_min);

			settings.ref_lla << ref_lat, ref_long, ref_alt; 
		
			//Make pointcloud svaver object
			mapper = new PointCloudMapper(settings);

			cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudMapperNode::pointCloudCallback, this, std::placeholders::_1));

			//set up location subs
			location_sub = this->create_subscription<atr_msgs::msg::StateEstimate>(location_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudMapperNode::locationCallback, this, std::placeholders::_1));
			raw_location_sub = this->create_subscription<atr_msgs::msg::LOCALIZATION>(raw_location_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudMapperNode::rawLocationCallback, this, std::placeholders::_1));
			raw_gps_sub = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(gps_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudMapperNode::rawGPSCallback, this, std::placeholders::_1));

			//todo: make a timer
			timer = this->create_wall_timer(std::chrono::milliseconds(rate), std::bind(&PointCloudMapperNode::timerCallback, this));
		}

		~PointCloudMapperNode()
		{
			this->mapper->saveMap();
			delete mapper;
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;  		//subscriber for the cloud to save
		rclcpp::Subscription<atr_msgs::msg::StateEstimate>::SharedPtr location_sub;  	//subscriber for a NED localizaiton result
		rclcpp::Subscription<atr_msgs::msg::LOCALIZATION>::SharedPtr raw_location_sub;  //subscriber for an lla localization result
		rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr raw_gps_sub;  	//subscriber for a novatel message
		
		rclcpp::TimerBase::SharedPtr timer;

		PointCloudMapper* mapper;
		PointCloudMapper::Settings settings;

		//Methods:
		/**
		 * @Function 	pointCloudCallback
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
		 */
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			//memcpy into a struct (parses the data into the struct values)
			// std::vector<pointcloud_utils::luminarPointstruct> cloud;
			// cloud.resize(msg->width);
			// std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);
		
			//call save to file
			// savePointsToFile(cloud);
		
			// this->times.push_back(msg->header.stamp);

			this->mapper->setCurrentCloud(msg);
		}

		void locationCallback(const atr_msgs::msg::StateEstimate::SharedPtr msg)
		{
			this->mapper->setCurrentLocation(msg);
		}

		void rawLocationCallback(const atr_msgs::msg::LOCALIZATION::SharedPtr msg)
		{
			this->mapper->setCurrentRawLocation(msg);
		}

		void rawGPSCallback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg)
		{
			this->mapper->setCurrentGPS(msg);
		}

		//make callback for timer, and call update and print map if param says to
		void timerCallback()
		{
			this->mapper->updateMap();
		}

};


int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMapperNode>());
    rclcpp::shutdown();
	return(0);
}