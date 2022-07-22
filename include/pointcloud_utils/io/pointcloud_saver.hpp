/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
 * Brief: a class to subscribe to and save-to-file pointcloud messages
 * File: pointcloud_saver.hpp
 */

#ifndef POINTCLOUD_SAVER_HPP
#define POINTCLOUD_SAVER_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <vector>
#include <fstream>
#include <sstream>

#include "pointcloud_utils/pointcloud_utils.hpp"

namespace pointcloud_utils
{

class
PointCloudSaver
{
public:
	PointCloudSaver(std::string filename, std::string file_extension);
	~PointCloudSaver();

	// *
	 // * @Function 	setInputTopic
	 // * @Param 		topic - name of cloud topic to save
	 // * @Return 		void
	 // * @Brief 		Re-sets the current input topic and subscription
	 // 
	// void setIntputTopic(std::string topic);

	/**
	 * @Function 	setCurrentCloud
	 * @Brief 		Manually sets the next cloud to process into the saver, and
	 * 				initiates the processing on this new cloud
	 * @Param		cloud - the point cloud to process
	 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
	 * @Return      void
	 */
	void setCurrentCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const std::string& filename="");

	/**
	 * @Function 	setCurrentCloud
	 * @Brief 		Manually sets the next cloud to process into the saver, and
	 * 				initiates the processing on this new cloud
	 * @Param		cloud - the point cloud to process
	 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
	 * @Return      void
	 */
	void setCurrentCloud(const sensor_msgs::msg::PointCloud2& cloud, const std::string& filename="");

	/**
	 * @Function 	setCurrentCloud
	 * @Brief 		Manually sets the next cloud to process into the saver, and
	 * 				initiates the processing on this new cloud
	 * @Param		cloud - the point cloud to process
	 * @Param 		time - time of the given cloud
	 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
	 * @Return      void
	 */
	void setCurrentCloud(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const double& time, const std::string& filename="");

	/**
 	 * @Function 	saveTimesToFile
 	 * @Param 		none
 	 * @Return 		void
 	 * @Brief 		Saves the time array to a default file
 	 */
	void saveTimesToFile();

private:
	std::string     input_topic;		//topic for the cloud to save
	rclcpp::Node* 	node; 				//node handle for subscription
	unsigned int 	filename_counter; 	//increments the save file name
	std::string 	filename_base; 		//base of the save file name
	std::string 	file_extension; 	//"." extension type of the save file
	// rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;  		//subscriber for the cloud to save
	std::mutex 		save_mutex; 		//protects the current write function from being interrupted

	std::vector<double> times; 	//holds the times for all files processed so far
	std::string time_file_name;			//Default name for the times file

	// /**
 // 	 * @Function 	pointCloudCallback
 // 	 * @Param 		msg - incoming data message
 // 	 * @Return 		void
 // 	 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 // 	 */
	// void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

	/**
 	 * @Function 	savePointsToFile
 	 * @Param 		cloud - cloud to save
 	 * @Return 		void
 	 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 	 */
	void savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud);
	
	/**
	 * @Function 	savePointsToFile
	 * @Param 		cloud - cloud to save to
	 * @Param 		filename - name of file to save to
	 * @Return 		void
	 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
	 */
	void savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const std::string& filename);

};

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_SAVER_HPP