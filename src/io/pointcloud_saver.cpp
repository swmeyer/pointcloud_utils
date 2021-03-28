/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
/* Brief: a class to subscribe to and save-to-file pointcloud messages
/* File: pointcloud_saver.cpp
*/

// --------------------------
#include "pointcloud_utils/io/pointcloud_saver.hpp"
// --------------------------

PointCloudSaver::PointCloudSaver(std::string topic, std::string filename, std::string file_extension, ros::NodeHandle& n):
	filename_counter(0)
{
	this->n = n;
	this->filename_base = filename;
	this->file_extension = file_extension;
	this->time_file_name = "times";
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
 * @Function 	setCurrentCloud
 * @Brief 		Manually sets the next cloud to process into the saver, and
 * 				initiates the processing on this new cloud
 * @Param		cloud - the point cloud to process
 * @Return      void
 */
void PointCloudSaver::setCurrentCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointstruct> cloud2;
	cloud2.resize(cloud->width);
	std::memcpy(&(cloud2[0]), &(cloud->data[0]), cloud->row_step);

	//call save to file
	savePointsToFile(cloud2);

	this->times.push_back(cloud->header.stamp);
}

/**
 * @Function 	saveTimesToFile
 * @Param 		none
 * @Return 		void
 * @Brief 		Saves the time array to a default file
 */
void PointCloudSaver::saveTimesToFile()
{
	//prepare file name
	std::stringstream sstream;
	sstream << filename_base << "_" << time_file_name << file_extension;
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
			file << "time_s\n"; //headers for csv
			for (ros::Time time : this->times)
			{
				file << time << ",\n";
			}
		} else
		{
			std::cout << "Filetype not suppported: " << file_extension << "\n";
		}
		
	} else
	{
		std::cout << "Could not open file: " << filename << "\n";
	}
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

	this->times.push_back(msg->header.stamp);
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
