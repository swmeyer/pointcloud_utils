/** Author: Stephanie Meyer swmeyer16@gmail.com 3 Feb 2020
 * Brief: a class to subscribe to and save-to-file pointcloud messages
 * File: pointcloud_saver.cpp
*/

// --------------------------
#include "pointcloud_utils/io/pointcloud_saver.hpp"
// --------------------------

namespace pointcloud_utils
{

PointCloudSaver::PointCloudSaver(std::string filename, std::string file_extension):
	filename_counter(0)
{
	// this->node = node;
	this->filename_base = filename;
	this->file_extension = file_extension;
	this->time_file_name = "times";
	// setIntputTopic(topic);
}

PointCloudSaver::~PointCloudSaver() 
{

}

// /**
//  * @Function 	setInputTopic
//  * @Param 		topic - name of cloud topic to save
//  * @Return 		void
//  * @Brief 		Re-sets the current input topic and subscription
//  */
// void PointCloudSaver::setIntputTopic(std::string topic)
// {
// 	while (save_mutex.try_lock()) {/*spin*/}

// 	cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudSaver::pointCloudCallback, this, std::placeholders::_1));

// 	save_mutex.unlock();
// }

/**
 * @Function 	setCurrentCloud
 * @Brief 		Manually sets the next cloud to process into the saver, and
 * 				initiates the processing on this new cloud
 * @Param		cloud - the point cloud to process
 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
 * @Return      void
 */
void PointCloudSaver::setCurrentCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const std::string& filename)
{
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointcloud_utils::luminarPointstruct> cloud2;
	cloud2.resize(cloud->width);
	std::memcpy(&(cloud2[0]), &(cloud->data[0]), cloud->row_step);

	double time = cloud->header.stamp.sec + (cloud->header.stamp.nanosec * 10e-10);

	this->setCurrentCloud(cloud2, time, filename);
}

/**
 * @Function 	setCurrentCloud
 * @Brief 		Manually sets the next cloud to process into the saver, and
 * 				initiates the processing on this new cloud
 * @Param		cloud - the point cloud to process
 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
 * @Return      void
 */
void PointCloudSaver::setCurrentCloud(const sensor_msgs::msg::PointCloud2& cloud, const std::string& filename)
{
	//memcpy into a struct (parses the data into the struct values)
	std::vector<pointcloud_utils::luminarPointstruct> cloud2;
	cloud2.resize(cloud.width);
	std::memcpy(&(cloud2[0]), &(cloud.data[0]), cloud.row_step);

	double time = cloud.header.stamp.sec + (cloud.header.stamp.nanosec * 10e-10);

	this->setCurrentCloud(cloud2, time, filename);
}

/**
 * @Function 	setCurrentCloud
 * @Brief 		Manually sets the next cloud to process into the saver, and
 * 				initiates the processing on this new cloud
 * @Param		cloud - the point cloud to process
 * @Param 		time - time of the given cloud
 * @Param 		filename - filename to save to. if empty, use a constructed default file name (default: empty)
 * @Return      void
 */
void PointCloudSaver::setCurrentCloud(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const double& time, const std::string& filename)
{
	std::cout << "Saving " << cloud.size() << " points to file " << filename << "\n";
	//call save to file
	if (filename == "")
	{
		this->savePointsToFile(cloud);
	} else
	{
		this->savePointsToFile(cloud, filename);
	}

	this->times.push_back(time);
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
			for (double time : this->times)
			{
				file.precision(17);
				file << std::fixed << time << ",\n";
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

// /**
//  * @Function 	pointCloudCallback
//  * @Param 		msg - incoming data message
//  * @Return 		void
//  * @Brief 		Reacts to incoming pointcloud messages. Calls the save-to-file function
//  */
// void PointCloudSaver::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
// 	//memcpy into a struct (parses the data into the struct values)
// 	std::vector<pointcloud_utils::luminarPointstruct> cloud;
// 	cloud.resize(msg->width);
// 	std::memcpy(&(cloud[0]), &(msg->data[0]), msg->row_step);

// 	//call save to file
// 	savePointsToFile(cloud);

// 	this->times.push_back(msg->header.stamp);
// }

/**
 * @Function 	savePointsToFile
 * @Param 		cloud - cloud to save to
 * @Return 		void
 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 */
void PointCloudSaver::savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud)
{
	while (save_mutex.try_lock()) {/*spin*/}

	//prepare filename
	std::stringstream sstream;
	sstream << filename_base << "_" << (int) ++filename_counter << file_extension;
	std::string filename = sstream.str();

	this->savePointsToFile(cloud, filename);
}

/**
 * @Function 	savePointsToFile
 * @Param 		cloud - cloud to save to
 * @Param 		filename - name of file to save to
 * @Return 		void
 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 */
void PointCloudSaver::savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const std::string& filename)
{
	//Open file
	std::ofstream file;
	file.open(filename);

	if (file.is_open())
	{
		std::cout << "Saving to file: " << filename << "\n";
		//Write to file
		if (file_extension == ".csv")
		{
			file << "x, y, z, intensity, ring, point_time\n"; //headers for csv
			for (pointcloud_utils::luminarPointstruct pt : cloud)
			{
				file << pt.x << "," 
				     << pt.y << ","
				     << pt.z << ","
				     << pt.intensity << ","
				     << pt.ring << ","
				     << pt.time << "\n";
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

} //end namespace pointcloud_utils
