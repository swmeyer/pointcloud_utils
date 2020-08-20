#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <sstream>

#include "pointcloud_utils/pointcloud_utils.hpp"

int filename_counter = 0;
unsigned long int timestamp;

std::string file_list_name = "points/innoviz_file_list.txt";

/**
 * @Function    savePointsToFile
 * @Param       cloud - cloud to save to
 * @Return      void
 * @Brief       Saves the given cloud to a default file with an incremented counter in the name
 */
void saveToFile(std::vector<pointcloud_utils::pointstruct>& cloud)
{
    std::string filename_base = "points/innoviz_points";
    std::string file_extension = ".xyz";

    //prepare filename
    std::stringstream sstream;
    sstream << filename_base << "_" << (int) filename_counter++  << "_" << std::to_string(timestamp);
    std::string file_no_extension = sstream.str();
    sstream << file_extension;
    std::string filename = sstream.str();

    //Open file
    std::ofstream file;
    file.open(filename);

    if (file.is_open())
    {
        std::cout << "Saving to file: " << filename << "\n";
        //Write to file
        if (file_extension == ".xyz")
        {
            file << "x, y, z, intensity,\n"; //headers for csv
            for (pointcloud_utils::pointstruct pt : cloud)
            {
                file << pt.x << "," 
                     << pt.y << ","
                     << pt.z << ","
                     << pt.intensity << ",\n";
            }
        } else
        {
            std::cout << "Filetype not suppported: " << file_extension << "\n";
        }
        
    } else
    {
        std::cout << "Could not open file: " << filename << "\n";
    }

    file.close();

    file.open(file_list_name, std::fstream::app);
    if (file.is_open())
    {
        file << (filename_counter - 1) << ", " << std::to_string(timestamp) << ", " << file_no_extension << ",\n";
    }
    file.close();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bag_parser");
    ros::NodeHandle n_("~");
    rosbag::Bag bag;

    std::string bagstring = "/home/stephanie/Documents/data/fp_bags_truckcomputer/day_2/day2_run1_part2_restamp.bag";
    std::cout << "Opening bag: " << bagstring << "\n";

    bag.open(bagstring);  // BagMode is Read by default
    
    for(rosbag::MessageInstance const m: rosbag::View(bag, rosbag::TopicQuery("/innoviz_points")))
    {
      sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
      if (i != nullptr)
      {
        if (!ros::ok())
        {
            break;
        }
        // if (i->header.frame_id != "innoviz")
        // {
        //     continue;
        // }

        //parse pointcloud message into a usable cloud
        std::vector<pointcloud_utils::pointstruct> cloud;
        cloud.resize(i->width);
        std::memcpy(&(cloud[0]), &(i->data[0]), i->row_step);
        std::cout << "Cloud parsed out to pointstruct\n";

        timestamp = i->header.stamp.toNSec();

        //save to file (open file, write, close)
        saveToFile(cloud);
      }

    }

    bag.close();

    return 0;
}
