/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
/* Brief: a class to subscribe to and save-to-file pointcloud messages
/* File: pointcloud_saver.cpp
*/

// --------------------------
#include "pointcloud_utils/pointcloud_saver.hpp"
// --------------------------

PointCloudSaver::PointCloudSaver(std::string topic, std::string filename, std::string file_extension, ros::NodeHandle& n):
	filename_counter(0)
{
	this->n = n;
	this->filename_base = filename;
	this->file_extension = file_extension;
	setIntputTopic(topic);
}

PointCloudSaver::~PointCloudSaver() {}

/**
 * @Function 	setInputTopic
 * @Param 		topic - name of cloud topic to save
 * @Return 		void
 * @Brief 		Re-sets the current input topic and subscription
 */
void PointCloudSaver::setIntputTopic(std::string topic)
{
	while (save_mutex.try_lock()) {/*spin*/}

	cloud_sub = n.subscribe(topic, 1, &PointCloudSaver::pointCloudCallback, this);

	save_mutex.unlock();
}

/**
 * @Function 	pointCloudCallback
 * @Param 		msg - incoming data message
 * @Return 		void
 * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
 */
void PointCloudSaver::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointstruct> cloud;
	cloud.resize(msg->width);
	std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);

	//call save to file
	savePointsToFile(cloud);
}

/**
 * @Function 	savePointsToFile
 * @Param 		cloud - cloud to save to
 * @Return 		void
 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 */
void PointCloudSaver::savePointsToFile(std::vector<pointstruct>& cloud)
{
	while (save_mutex.try_lock()) {/*spin*/}

	//prepare filename
	std::stringstream sstream;
	sstream << filename_base << "_" << (int) ++filename_counter << file_extension;
	std::string filename = sstream.str();

	//Open file
	std::ofstream file;
	file.open(filename);

	if (file.is_open())
	{
		std::cout << "Saving to file: " << filename << "\n";
		//Write to file
		if (file_extension == ".csv")
		{
			file << "x, y, z, intensity, ring\n"; //headers for csv
			for (pointstruct pt : cloud)
			{
				file << pt.x << "," 
				     << pt.y << ","
				     << pt.z << ","
				     << pt.intensity << ","
				     << pt.ring << "\n";
			}
		} else
		{
			std::cout << "Filetype not suppported: " << file_extension << "\n";
		}
		
	} else
	{
		std::cout << "Could not open file: " << filename << "\n";
	}

	save_mutex.unlock();
}
