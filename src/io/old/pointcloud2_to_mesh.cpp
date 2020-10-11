/** 
 * Author: Stephanie Meyer swm0022@tigermail.auburn.edu 1-30-19
 *
 * This program acts as a test of pcl-equipped techniques for converting from
 * ROS pointcloud to a .ply mesh file.
 * 
 * PCL techniques: http://pointclouds.org/documentation/tutorials/greedy_projection.php
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class
pointcloud2ToMesh
{
	public:
	pointcloud2ToMesh(ros::NodeHandle pnh);
	~pointcloud2ToMesh() {}

	bool convertToMesh();
	void getCloud();

	bool save_all;


	private:
	bool has_cloud_;
	ros::NodeHandle nh_;
	std::mutex cloud_mutex_;
	sensor_msgs::PointCloud2 cloud_in_;
	std::string output_file_;
	std::string cloud_topic_;
	int file_num_;
	void getCloud(std::string cloud_topic);
	void cloudInCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

/** ------------------------------------------------------------------------------------- */

pointcloud2ToMesh::pointcloud2ToMesh(ros::NodeHandle pnh):
	file_num_(0)
{
	nh_ = pnh;
	std::string cloud_topic;
	nh_.param<std::string>("cloud_in_topic", cloud_topic_, "/terrain");
	nh_.param<std::string>("output_filename", output_file_, "cloud.ply");
	nh_.param<bool>("save_all", this->save_all, false);

	// while(ros::ok())
	// {
	// 	// ros::spinOnce();
	// 	getCloud(cloud_topic);
	// 	if (!save_all_) break;
	// }
}

/** ------------------------------------------------------------------------------------- */

/** Convert the current pointcloud to a mesh object */
bool pointcloud2ToMesh::convertToMesh()
{
	if (!has_cloud_)
	{
		ROS_INFO_STREAM("**WARNING**: No pointcloud currently loaded. Mesh not generated.");
		return false;
	}
	// ROS_INFO_STREAM("Converting to file type.");

	pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud_in_,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  std::string write_path = output_file_;
  if (this->save_all)
  {
    std::string delimiter = "/";
    size_t next_index = output_file_.find(delimiter); //index of the first / in the filename
    std::string file_name = output_file_;
    std::string path = "";
    while (next_index != std::string::npos && ros::ok())
    {
      path += file_name.substr(0, next_index + 1);
      file_name = file_name.substr(next_index + 1,file_name.length());
      next_index = file_name.find(delimiter);
    }

    delimiter = ".";
    std::string extension;
    size_t index = file_name.find(delimiter);
    extension = file_name.substr(index,file_name.length());
    file_name = file_name.substr(0, index);

    // ROS_INFO_STREAM("path, file, extension:" << path << "  " << file_name << "  " << extension);

  	write_path = path + file_name + std::to_string(file_num_++) + extension;
  }

  ROS_INFO_STREAM("Writing cloud to file: " << write_path);
	pcl::io::savePLYFileBinary(write_path, *cloud);

  // int output_type = 3; //Not ASCII (0), Binary (1), or Binary_compressed (2)
  // savePointCloud (&pcl_pc2, "mesh.ply", output_type); //Note: can also save as .pcd here
  
  
 //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

	// // Normal estimation*
 //  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
 //  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
 //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 //  tree->setInputCloud (cloud);
 //  n.setInputCloud (cloud);
 //  n.setSearchMethod (tree);
 //  n.setKSearch (20);
 //  n.compute (*normals);
 //  //* normals should not contain the point normals + surface curvatures

 //  // Concatenate the XYZ and normal fields*
 //  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
 //  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
 //  //* cloud_with_normals = cloud + normals

 //  // Create search tree*
 //  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
 //  tree2->setInputCloud (cloud_with_normals);

 //  // Initialize objects
 //  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
 //  pcl::PolygonMesh triangles;

 //  // Set the maximum distance between connected points (maximum edge length)
 //  gp3.setSearchRadius (0.025);

 //  // Set typical values for the parameters
 //  gp3.setMu (2.5);
 //  gp3.setMaximumNearestNeighbors (100);
 //  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
 //  gp3.setMinimumAngle(M_PI/18); // 10 degrees
 //  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
 //  gp3.setNormalConsistency(false);

 //  // Get result
 //  gp3.setInputCloud (cloud_with_normals);
 //  gp3.setSearchMethod (tree2);
 //  gp3.reconstruct (triangles);

 //  // Additional vertex information
 //  std::vector<int> parts = gp3.getPartIDs();
 //  std::vector<int> states = gp3.getPointStates();
  
 //  pcl::io::saveVTKFile ("mesh.vtk", triangles);
	// pcl::io::convertVtkToPly("mesh.vtk", "mesh.ply");
	has_cloud_ = false;
	return true;
}

/** ------------------------------------------------------------------------------------- */

/**
 *	Import pointcloud
 */
void pointcloud2ToMesh::getCloud(std::string cloud_topic)
{
  this->has_cloud_ = false;
  ros::Subscriber cloud_in_sub = this->nh_.subscribe(cloud_topic, 1, &pointcloud2ToMesh::cloudInCallback, this);
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Waiting for cloud to be published.\n");
  //has_cloud_ will be false until the cloudInCallback has been run on a subscribed message
  while(!this->has_cloud_ && ros::ok()) 
  {
    ros::spinOnce();
    //TODO: add a sleep
  }

  //The cloud might still be reset to a new incoming message at this point. Wait until the 
  // cloud-setting process in cloudInCallback has completed for the current cloud, then turn 
  // off the cloudInCallback subscription thread
  while(!cloud_mutex_.try_lock() && ros::ok())
  {
    //spin until lock is acquired
  }
  ROS_INFO_STREAM("Cloud acquired.\n");
  cloud_in_sub.shutdown();
  cloud_mutex_.unlock();
}

/** ------------------------------------------------------------------------------------- */

void pointcloud2ToMesh::getCloud()
{
  this->has_cloud_ = false;
  ros::Subscriber cloud_in_sub = this->nh_.subscribe(cloud_topic_, 1, &pointcloud2ToMesh::cloudInCallback, this);
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Waiting for cloud to be published.\n");
  //has_cloud_ will be false until the cloudInCallback has been run on a subscribed message
  while(!this->has_cloud_ && ros::ok()) 
  {
    ros::spinOnce();
  }

  //The cloud might still be reset to a new incoming message at this point. Wait until the 
  // cloud-setting process in cloudInCallback has completed for the current cloud, then turn 
  // off the cloudInCallback subscription thread
  while(!cloud_mutex_.try_lock() && ros::ok())
  {
    //spin until lock is acquired
  }
  ROS_INFO_STREAM("Cloud acquired.\n");
  cloud_in_sub.shutdown();
  cloud_mutex_.unlock();
}

/** ------------------------------------------------------------------------------------- */

void pointcloud2ToMesh::cloudInCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
  while(!cloud_mutex_.try_lock())
  {
    //spin until lock is acquired
  }
  this->cloud_in_ = *msg;
  this->has_cloud_ = true;
  cloud_mutex_.unlock();
}

/** ------------------------------------------------------------------------------------- */

int main (int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud2_to_mesh");
	ros::NodeHandle nh_("~");

	//Create new instance of mesh generator
	pointcloud2ToMesh mesh_generator(nh_);
	ROS_INFO_STREAM("File saver intitiated.");

	while (ros::ok())
	{
		// ROS_INFO_STREAM("File saver cycle.");
		mesh_generator.getCloud();
		mesh_generator.convertToMesh();
		if (!mesh_generator.save_all) break;
	}


	ROS_INFO_STREAM("Finished saving to file.");

  return (0);
}