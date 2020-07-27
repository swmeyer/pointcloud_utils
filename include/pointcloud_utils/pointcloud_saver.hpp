/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
 * Brief: a class to subscribe to and save-to-file pointcloud messages
 * File: pointcloud_saver.hpp
 */

#ifndef POINTCLOUD_SAVER_HPP
#define POINTCLOUD_SAVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <vector>
#include <fstream>
#include <sstream>

class
PointCloudSaver
{
public:
	PointCloudSaver(std::string topic, std::string filename, std::string file_extension, ros::NodeHandle& n);
	~PointCloudSaver();

	/**
	 * @Function 	setInputTopic
	 * @Param 		topic - name of cloud topic to save
	 * @Return 		void
	 * @Brief 		Re-sets the current input topic and subscription
	 */
	void setIntputTopic(std::string topic);

private:
	std::string     input_topic;		//topic for the cloud to save
	ros::NodeHandle n; 					//node handle for subscription
	unsigned char 	filename_counter; 	//increments the save file name
	std::string 	filename_base; 		//base of the save file name
	std::string 	file_extension; 	//"." extension type of the save file
	ros::Subscriber cloud_sub;  		//subscriber for the cloud to save
	std::mutex 		save_mutex; 		//protects the current write function from being interrupted

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

	/**
 	 * @Function 	pointCloudCallback
 	 * @Param 		msg - incoming data message
 	 * @Return 		void
 	 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 	 */
	void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

	/**
 	 * @Function 	savePointsToFile
 	 * @Param 		cloud - cloud to save to
 	 * @Return 		void
 	 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 	 */
	void savePointsToFile(std::vector<pointstruct>& cloud);

};

#endif //end ifndef POINTCLOUD_SAVER_HPP