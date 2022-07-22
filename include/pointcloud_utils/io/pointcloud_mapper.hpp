/** Author: Stephanie Meyer swmeyer16@gmail.com 15 Feb 2022
 * Brief: a class to build a single point map out of localization estimates and incoming point clouds
 * File: pointcloud_mapper.hpp
 */

#ifndef POINTCLOUD_MAPPER_HPP
#define POINTCLOUD_MAPPER_HPP

//-----------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <vector>
#include <queue>
#include <fstream>
#include <sstream>

#include "pointcloud_utils/pointcloud_utils.hpp"
#include "pointcloud_utils/pointcloud_utils_impl.hpp"
#include "atr_msgs/msg/state_estimate.hpp"
#include "atr_msgs/msg/localization.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "pointcloud_utils/io/frame_conversions.h"
//-----------------------------------------------

class
PointCloudMapper
{
public:
	typedef struct Settings
	{
		std::string 	pointfilename_base; 		//base of the points save file names
		std::string     locationfilename; 			//location save file name
		std::string     rawlocationfilename; 		//raw location save file name
		std::string     gpsfilename; 				//gps data save file name
		std::string 	mapfilename; 				//map save file name

		double time_tolerance; 	//[s] the acceptable difference in time between the point cloud timestamp and its matched locaiton message
		double time_offset; 	//[s] an offset to be added to the location time (to account for static time offset between the timestamps)

		bool use_raw_location;
		bool use_gps;

		bool save_transformed_points; //if true, save transformed points instead of raw

		// bool save_points;
		// bool save_locations;
		// bool save_map;

		double altitude_max; //Max lla altitude to consider valid

		Eigen::Matrix<double, 3, 1> ref_lla; //reference lla for conversion

		double downsample_percent; //in decimal form, the percent to downsample each point cloud before using it in the map
		//Single point frame bounds in [m]:
		double x_min;
		double x_max;
		double y_min;
		double y_max;
		double z_min;
		double z_max;

		//Ego removal box, in [m]:
		double ego_x_max;
		double ego_x_min;
		double ego_y_max;
		double ego_y_min;
		double ego_z_max;
		double ego_z_min;
	} Settings;

	PointCloudMapper(const Settings& settings);
	~PointCloudMapper();


	/**
	 * @Function 	setCurrentCloud
	 * @Brief 		Manually sets the next cloud to process into the saver, and
	 * 				initiates the processing on this new cloud
	 * @Param		cloud - the point cloud to process
	 * @Return      void
	 */
	void setCurrentCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

	/**
	 * @Function 	setCurrentLocation
	 * @Brief 		updates the saved location estimate with the given location
	 * @Param 		location - NED location update
	 * @Return 		void
	 */
	void setCurrentLocation(const atr_msgs::msg::StateEstimate::SharedPtr location);

	/**
	 * @Function 	setCurrentRawLocation
	 * @Brief 		updates the saved location estimate with the given location
	 * @Param 		location - lla location update
	 * @Return 		void
	 */
	void setCurrentRawLocation(const atr_msgs::msg::LOCALIZATION::SharedPtr location);

	/**
	 * @Function 	setCurrentGPS
	 * @Brief 		updates the saved gps data
	 * @Param 		location - gps data
	 * @Return 		void
	 */
	void setCurrentGPS(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location);

	/**
 	 * @Function 	saveMap
 	 * @Param 		none
 	 * @Return 		void
 	 * @Brief 		Saves the map to file
 	 */
	void saveMap();

	/**
	 * @function 	updateMap
	 * @brief 		Using the current pointcloud and location data, update the stored point map
	 * @param 		none
	 * @return 		void
	 */
	void updateMap();

private:
	const double deg2rad = M_PI / 180;

	Settings settings; //Struct that holds the behavior settings for the class

	unsigned int 	pointframe_counter; 		//counts the number of pointclouds processed
	bool has_new_cloud;
	bool has_location;
	bool first_location;	 //true until first locaiton is saved
	bool first_raw_location; //true until first raw locaiton is saved
	bool first_gps;			 //true until first gps data is saved

	WgsConversions* frame_conversions;

	std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pending_clouds;
	std::queue<atr_msgs::msg::StateEstimate> pending_locations;
	// std::queue<atr_msgs::msg::StateEstimate::SharedPtr> pending_raw_locations;

	atr_msgs::msg::StateEstimate current_location;

	std::vector<pointcloud_utils::luminarPointstruct> map;

	std::mutex 		pointsave_mutex; 			//protects the current write function from being interrupted
	std::mutex 		locationsave_mutex; 		//protects the current write function from being interrupted
	std::mutex 		locationrawsave_mutex; 		//protects the current write function from being interrupted
	std::mutex 		gpssave_mutex;		 		//protects the current write function from being interrupted

	/**
	 * @Function 	transformLocationToNED
	 * @Brief 		converts the given gps data to NED
	 * @Param 		location - the location to convert
	 * @Return 		the converted NED location
	*/
	atr_msgs::msg::StateEstimate transformLocationToNED(const atr_msgs::msg::LOCALIZATION::SharedPtr location);
	atr_msgs::msg::StateEstimate transformLocationToNED(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location);
	atr_msgs::msg::StateEstimate transformLocationToNED(const atr_msgs::msg::LOCALIZATION& location);

	// /**
	//  * @Function 	getCurrentLocation
	//  * @Param 		header - contians the time stamp of the current cloud
	//  * @Param 		locaiton - place to store the retrieved location
	//  * @Return 		void
	//  * @Brief 		Searches for the location message with the closest time match to the given header and returns the selected location
	//  */
	// void getCurrentLocaiton(const std_msgs::msg::Header& header, atr_msgs::msg::StateEstimate::SharedPtr location);

	/**
	 * @Function 	downsampleCloud
	 * @Brief 		takes the given cloud and pass-through filters it to the settings bounds as well as downsamples the point density
	 * @Param 		cloud - cloud to downsample and cut
	 * @Param 		downsampled_cloud - place to store returned cloud
	 * @Return 		void
	 */
	void downsampleCloud(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, std::vector<pointcloud_utils::luminarPointstruct>& downsampled_cloud);

	/**
 	 * @Function 	savePointsToFile
 	 * @Param 		cloud - cloud to save
 	 * @Return 		void
 	 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 	 */
	void savePointsToFile(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
	void savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud);
	void savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const std::string& filename);

	/**
	 * @Function 	saveLocationToFile
	 * @Param 		location
	 * @Return		void
	 * @Brief 		adds the current location to a csv file
	 */ 
	void saveLocationToFile(const atr_msgs::msg::StateEstimate::SharedPtr location);

	/**
	 * @Function 	saveRawLocationToFile
	 * @Param 		location
	 * @Return		void
	 * @Brief 		adds the current location to a csv file
	 */ 
	void saveRawLocationToFile(const atr_msgs::msg::LOCALIZATION::SharedPtr location);

	/**
	 * @Function 	saveGPSToFile
	 * @Param 		location
	 * @Return		void
	 * @Brief 		adds the current location to a csv file
	 */ 
	void saveGPSToFile(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location);


	//Frame conversion helpers
	void e2a(double vout[3], Eigen::Matrix<double, 3, 1>& v);
	void a2e( Eigen::Matrix<double, 3, 1>& vout, double v[3]);
	Eigen::Matrix<double, 3, 1> convertlla2ned(Eigen::Matrix<double, 3, 1>& lla, Eigen::Matrix<double, 3, 1>& ref_lla);

};

#endif //end ifndef POINTCLOUD_MAPPER_HPP