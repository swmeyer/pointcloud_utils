/** Author: Stephanie Meyer swmeyer16@gmail.com 15 Feb 2022
 * Brief: a class to build a single point map out of localization estimates and incoming point clouds
 * File: pointcloud_mapper.cpp
 */

// --------------------------
#include "pointcloud_utils/io/pointcloud_mapper.hpp"
// --------------------------

PointCloudMapper::PointCloudMapper(const Settings& settings):
	pointframe_counter(0),
	has_new_cloud(false),
	has_location(false),
	first_location(true),
	first_raw_location(true),
	first_gps(true)
{
	this->settings = settings;	
	this->frame_conversions = new WgsConversions();
}

PointCloudMapper::~PointCloudMapper()
{
	delete this->frame_conversions;
}

/**
 * @Function 	setCurrentCloud
 * @Brief 		Manually sets the next cloud to process into the saver, and
 * 				initiates the processing on this new cloud
 * @Param		cloud - the point cloud to process
 * @Return      void
 */
void PointCloudMapper::setCurrentCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
	std::cout << "Cloud received\n";
	if (!this->has_location)
	{
		std::cout << "Location not acquired. not updating map\n";
		return; //Can't do anything with this cloud yet
	}

	//add this cloud to queue
	this->pending_clouds.push(cloud);
	this->pending_locations.push(current_location);

	this->has_new_cloud = true;

	if (!(this->settings.save_transformed_points))
	{
		this->savePointsToFile(cloud);
	}
}

/**
 * @Function 	setCurrentLocation
 * @Brief 		updates the saved location estimate with the given location
 * @Param 		location - NED location update
 * @Return 		void
 */
void PointCloudMapper::setCurrentLocation(const atr_msgs::msg::StateEstimate::SharedPtr location)
{
	//add this location to the queue
	// this->pending_locations.push(location);
	this->current_location = *location;
	this->has_location = true;
	this->saveLocationToFile(location);
}

/**
 * @Function 	setCurrentRawLocation
 * @Brief 		updates the saved location estimate with the given location
 * @Param 		location - NED location update
 * @Return 		void
 */
void PointCloudMapper::setCurrentRawLocation(const atr_msgs::msg::LOCALIZATION::SharedPtr location) //TODO: change to novatel gps msg
{
	// std::cout << "received new raw location\n";

	// settings.use_raw_location = true;
	std::cout << "Use raw? " << this->settings.use_raw_location << "\n";
	if (this->settings.use_raw_location)
	{
		this->current_location = transformLocationToNED(location);
		this->has_location = true;
	}
	
	this->saveRawLocationToFile(location);
}

/**
 * @Function 	setCurrentGPS
 * @Brief 		updates the saved gps data
 * @Param 		location - gps data
 * @Return 		void
 */
void PointCloudMapper::setCurrentGPS(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location)
{
	std::cout << "Use gps? " << this->settings.use_gps << "\n";
	if (this->settings.use_gps)
	{
		this->current_location = transformLocationToNED(location);
		this->has_location = true;
	}

	this->saveGPSToFile(location);
}

/**
 * @Function 	transformLocationToNED
 * @Brief 		converts the given gps data to NED
 * @Param 		location - the location to convert
 * @Return 		the converted NED location
 */
atr_msgs::msg::StateEstimate PointCloudMapper::transformLocationToNED(const atr_msgs::msg::LOCALIZATION::SharedPtr location)
{
	return this->transformLocationToNED(*location);
}
atr_msgs::msg::StateEstimate PointCloudMapper::transformLocationToNED(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location)
{
	atr_msgs::msg::LOCALIZATION loc2; 
	loc2.header = location->header;
	loc2.latitude = location->latitude;
	loc2.longitude = location->longitude;
	loc2.height = location->height;
	loc2.roll = location->roll;
	loc2.pitch = location->pitch;
	loc2.azimuth = location->azimuth;

	return this->transformLocationToNED(loc2);
}
atr_msgs::msg::StateEstimate PointCloudMapper::transformLocationToNED(const atr_msgs::msg::LOCALIZATION& location)
{
	//convert from gps to NED

	// std::cout << "Converting to NED\n";
	double lat = location.latitude;
	double lon = location.longitude;
	double alt = location.height;
    
	double roll    = location.roll;
	double pitch   = location.pitch;
	double azimuth = location.azimuth; //Degreeeees! 
	// std::cout << "Finished acquiring members\n";   
    
	// Convert to local NED
	// Eigen::Matrix3f Rx = rotx(-roll);
	// Eigen::Matrix3f Ry = roty(-pitch);
	// Eigen::Matrix3f Rz = rotz(azimuth - 90); //TODO: is this in degrees??
    
 // 	Eigen::Matrix3f Rb2n = (Rx * Ry * Rz).inverse(); //TODO - eigen format?
    
 //    [e, n, u] = geodetic2NED(lat, lon, alt, ref_lat, ref_lon, ref_alt, wgs84Ellipsoid); //TODO: replace this with whatever happens in localization
 //    NED = [e; n; u];
 	Eigen::Matrix<double, 3, 1> lla;
 	lla << lat, lon, alt;
 	Eigen::Matrix<double, 3, 1> ned;

 	// std::cout << "ready to convert from lla to ned\n";
 	ned = convertlla2ned(lla, this->settings.ref_lla) ;
 	// std::cout << "converted from lla to ned\n";

	atr_msgs::msg::StateEstimate location_NED;
	// location_NED->header = location->header;
	// std::cout << "trouble accessing members? true if silence\n";
	location_NED.x_cg_m = ned[0];
	// std::cout << "first member accessed\n";
	location_NED.y_cg_m = ned[1];
	location_NED.z_cg_m = ned[2];

	location_NED.vx_cg_mps = roll; //Note: re-using this field improperly
	location_NED.vy_cg_mps = pitch;
	location_NED.psi_cg_rad = azimuth * this->deg2rad; 

	// std::cout << "Finished converting to NED. about to check for alt max\n";

	if (alt > this->settings.altitude_max)
    {
    	std::cout << "Zeroing transform at alt " << alt << "\n";
    	//zero out the result, if the measurement is invalid
    	location_NED.x_cg_m = 0;
		location_NED.y_cg_m = 0;
		location_NED.z_cg_m = 0;
		
		location_NED.vx_cg_mps = 0; //Note: re-using this field improperly
		location_NED.vy_cg_mps = 0;
		location_NED.psi_cg_rad = 0; 
    }

	return location_NED;
}

/**
 * @Function 	saveMap
 * @Param 		none
 * @Return 		void
 * @Brief 		Saves the map to file
 */
void PointCloudMapper::saveMap()
{
	this->savePointsToFile(this->map, this->settings.mapfilename);
}

/**
 * @function 	updateMap
 * @brief 		Using the current pointcloud and location data, update the stored point map
 * @param 		none
 * @return 		void
 */
void PointCloudMapper::updateMap()
{
	// sensor_msgs::PointCloud2 cloud_msg = this->pending_clouds.//get the next cloud from the queue
	// atr_msgs::msg::StateEstimate localization_msg = this->pending_locations //get the next location from the queue
	/**
	 * 1. Convert the point cloud into a matrix of positions
	 * 2. Convert the current NED location into a transform matrix
	 * 3. Transform the positions matrix by the transform matrix
	 * 4. Add the transformed positions as points into the point map
	 */

	if (has_new_cloud)
	{
		std::cout << "Updating map\n";
		has_new_cloud = false;
		//update map

		//Get cloud and location
		sensor_msgs::msg::PointCloud2::SharedPtr current_cloud = this->pending_clouds.front();
		this->pending_clouds.pop();

		std::vector<pointcloud_utils::luminarPointstruct> cloud2;
		pointcloud_utils::convertFromPointCloud2(current_cloud, cloud2);
		std::vector<pointcloud_utils::luminarPointstruct> downsampled_cloud;
		this->downsampleCloud(cloud2, downsampled_cloud);
		std::cout << "return down cloud size: " << downsampled_cloud.size() << "\n";

		atr_msgs::msg::StateEstimate current_location;
		current_location = this->pending_locations.front();
		this->pending_locations.pop();
		// getCorrespondingLocation(current_cloud->header, current_location);

		//Rotate Cloud
		//NOTE: localization is 2D NED -- //TODO: this probably isn't going to work. We'll need the full gps instead, and to transform it to NED
		pointcloud_utils::Transform transform;
		transform.roll = 0;
		transform.pitch = 0;
		transform.yaw = -current_location.psi_cg_rad;
		transform.x = current_location.x_cg_m;
		transform.y = -current_location.y_cg_m;
		transform.z = -current_location.z_cg_m;

		if (this->settings.use_raw_location)
		{
			//pull pitch, roll from reused fields
			transform.roll = current_location.vx_cg_mps;
			transform.pitch = -current_location.vy_cg_mps;
		}

		// std::cout << "cloud first point: " << cloud2[0].x << ", " << cloud2[0].y << "\n";

		// pointcloud_utils::transformCloud(cloud2, transform, false);
		pointcloud_utils::transformCloud(downsampled_cloud, transform, false);

		if (this->settings.save_transformed_points)
		{
			this->savePointsToFile(downsampled_cloud);
		}

		// std::cout << "transformed cloud first point: " << cloud2[0].x << ", " << cloud2[0].y << "\n";


		//Add cloud into current map
		// int old_map_size = this->map.size();
		// this->map.resize(this->map.size() + cloud2.size());
		// memcpy(&(map[old_map_size]), &(cloud2[0]), cloud2.size() * sizeof(cloud2[0]));

		int old_map_size = this->map.size();
		this->map.resize(this->map.size() + downsampled_cloud.size());
		memcpy(&(map[old_map_size]), &(downsampled_cloud[0]), downsampled_cloud.size() * sizeof(downsampled_cloud[0]));
		
		// int index = this->map.size() - 1; 
		// std::cout << "Map last point: " << this->map[index].x << ", " << this->map[index].y << "\n";

	} else
	{
		return; //No clouds to add
	}
}

// /**
//  * @Function 	getCurrentLocation
//  * @Param 		header - contians the time stamp of the current cloud
//  * @Param 		locaiton - place to store the retrieved location
//  * @Return 		void
//  * @Brief 		Searches for the location message with the closest time match to the given header and returns the selected location
//  */
// void PointCloudMapper::getCurrentLocaiton(const std_msgs::msg::Header& header, atr_msgs::msg::StateEstimate::SharedPtr location)
// {
// 	double time1 = header.stamp.sec + header.stamp.nanosec * 10^-9;

// 	for (uint i = 0; i < this->pending_locations.size(); i++)
// 	{
// 		atr_msgs::msg::StateEstimate::SharedPtr loc = this->pending_locations.front();

// 		double time2 = loc->header.sec + loc->header.nanosec * 10^-9 + this->settings.time_offset;

// 		if (pointcloud_utils::inTolerance(point_time, loc_time, this->settings.time_tolerance))
// 		{
// 			//Time match found!

// 			return loc;

// 		} else
// 		{
// 			this->pending_locations.pop();
// 		}
// 	}
// }

/**
 * @Function 	downsampleCloud
 * @Brief 		takes the given cloud and pass-through filters it to the settings bounds as well as downsamples the point density
 * @Param 		cloud - cloud to downsample and cut
 * @Param 		downsampled_cloud - place to store returned cloud
 * @Return 		void
 */
void PointCloudMapper::downsampleCloud(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, std::vector<pointcloud_utils::luminarPointstruct>& downsampled_cloud)
{
	std::cout << "\ndownsampling from cloud size " << cloud.size() << "\n";
	// std::vector<pointcloud_utils::luminarPointstruct> downsampled_cloud;
	downsampled_cloud.clear();

	int num_skip_total = cloud.size() * this->settings.downsample_percent;
	int num_remaining = cloud.size() - num_skip_total;
	int num_skip = cloud.size() / num_remaining;
	double full_num_skip = (cloud.size() / num_remaining) - num_skip;
	double remaining_skip = 0.0;
	std::cout << "Num skipping: " << full_num_skip << "\n";
	for (uint i = 0; i < cloud.size(); i+= num_skip)
	{
		pointcloud_utils::luminarPointstruct pt = cloud[i];
		if (pt.x > settings.x_max || pt.x < settings.x_min ||
			pt.y > settings.y_max || pt.y < settings.y_min ||
			pt.z > settings.z_max || pt.z < settings.z_min)
		{
			continue; //point out of bounds
		}
		if (pt.x < settings.ego_x_max && pt.x > settings.ego_x_min &&
			pt.y < settings.ego_y_max && pt.y > settings.ego_y_min &&
			pt.z < settings.ego_z_max && pt.z > settings.ego_z_min)
		{
			continue; //point belongs to ego
		}

		downsampled_cloud.push_back(pt);

		remaining_skip += full_num_skip;
		if (remaining_skip >= 1)
		{
			i++;
			remaining_skip -= 1.0;
		}
	}

	std::cout << "Down cloud size: " << downsampled_cloud.size() << "\n";
}

/**
 * @Function 	savePointsToFile
 * @Param 		cloud - cloud to save to
 * @Return 		void
 * @Brief 		Saves the given cloud to a default file with an incremented counter in the name
 */
void PointCloudMapper::savePointsToFile(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
	std::vector<pointcloud_utils::luminarPointstruct> cloud2;
	pointcloud_utils::convertFromPointCloud2(cloud, cloud2);

	this->savePointsToFile(cloud2);
}
void PointCloudMapper::savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud)
{
	//prepare filename
	std::stringstream sstream;
	sstream << this->settings.pointfilename_base << "_" << (int) ++this->pointframe_counter << ".csv";
	std::string filename = sstream.str();

	this->savePointsToFile(cloud, filename);
}
void PointCloudMapper::savePointsToFile(const std::vector<pointcloud_utils::luminarPointstruct>& cloud, const std::string& filename)
{
	while (pointsave_mutex.try_lock()) {/*spin*/}

	//Open file
	std::ofstream file;
	file.open(filename);

	if (file.is_open())
	{
		std::cout << "Saving to file: " << filename << "\n";
		//Write to file
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
		std::cout << "Could not open file: " << filename << "\n";
	}

	pointsave_mutex.unlock();
}


/**
 * @Function 	saveLocationToFile
 * @Param 		location
 * @Return		void
 * @Brief 		adds the current location to a csv file
 */ 
void PointCloudMapper::saveLocationToFile(const atr_msgs::msg::StateEstimate::SharedPtr location)
{
	while (locationsave_mutex.try_lock()) {/*spin*/}

	//Open file
	std::ofstream file;
	file.open(this->settings.locationfilename, std::ios_base::app);

	if (file.is_open())
	{
		// std::cout << "Saving to file: " << this->settings.locationfilename << "\n";
		//TODO: save content to file
		//Write to file

		//members:
		//uint64 time_ns
		//float64 x_cg_m
		//float64 y_cg_m
		//float64 z_cg_m
		//float64 psi_cg_rad
		//float64 vx_cg_mps
		//float64 vy_cg_mps
		//float64 dpsi_cg_radps
		//float64 beta_cg_rad
		//float64 ax_cg_mps2
		//float64 ay_cg_mps2
		//float64 kappa_radpm
		//float64 accuracy_pos_x_m
		//float64 accuracy_pos_y_m
		//float64 accuracy_pos_z_m
		//float64 accuracy_pos_psi_rad
		//float64 accuracy_vx_mps
		//float64 accuracy_vy_mps
		//float64 accuracy_dpsi_radps
		//uint8 status

		if (this->first_location)
		{
			file << "time_ns, x_cg_m, y_cg_m, z_cg_m, psi_cg_rad\n"; //headers for csv
			this->first_location = false;
		}
		file << location->time_ns << "," <<
				location->x_cg_m << "," <<
				location->y_cg_m << "," <<
				location->z_cg_m << "," <<
				location->psi_cg_rad << ",\n";
		
	} else
	{
		std::cout << "Could not open file: " << this->settings.locationfilename << "\n";
	}

	locationsave_mutex.unlock();
}

/**
 * @Function 	saveRawLocationToFile
 * @Param 		location
 * @Return		void
 * @Brief 		adds the current location to a csv file
 */ 
void PointCloudMapper::saveRawLocationToFile(const atr_msgs::msg::LOCALIZATION::SharedPtr location)
{
	while (locationrawsave_mutex.try_lock()) {/*spin*/}

	//Open file
	std::ofstream file;
	file.open(this->settings.rawlocationfilename, std::ios_base::app);

	if (file.is_open())
	{
		// std::cout << "Saving to file: " << this->settings.rawlocationfilename << "\n";
		//Write to file

		//Members:
		//std_msgs/Header  header 
		//float64          latitude 
		//float64          longitude 
		//float64          height
		//float64          north_velocity 
		//float64          east_velocity 
		//float64          up_velocity
		//float64          roll
		//float64          pitch
		//float64          azimuth
		//float64          side_slip
		if (this->first_raw_location)
		{
			file << "time_s, time_ns, latitude, longitude, height, roll, pitch, azimuth,\n"; //headers for csv
			this->first_raw_location = false;
		}
		file << location->header.stamp.sec << "," <<
				location->header.stamp.nanosec << "," <<
				location->latitude << "," <<
				location->longitude << "," <<
				location->height << "," <<
				location->roll << "," <<
				location->pitch << "," <<
				location->azimuth << ",\n";		
	} else
	{
		std::cout << "Could not open file: " << this->settings.rawlocationfilename << "\n";
	}

	file.close();

	locationrawsave_mutex.unlock();
}

/**
 * @Function 	saveGPSToFile
 * @Param 		location
 * @Return		void
 * @Brief 		adds the current location to a csv file
 */ 
void PointCloudMapper::saveGPSToFile(const novatel_oem7_msgs::msg::INSPVA::SharedPtr location)
{
	while (gpssave_mutex.try_lock()) {/*spin*/}

	//Open file
	std::ofstream file;
	file.open(this->settings.gpsfilename, std::ios_base::app);

	if (file.is_open())
	{
		// std::cout << "Saving to file: " << this->settings.gpsfilename << "\n";
		//Write to file

		//Members:
		//std_msgs/Header  header 
		//float64          latitude 
		//float64          longitude 
		//float64          height
		//float64          north_velocity 
		//float64          east_velocity 
		//float64          up_velocity
		//float64          roll
		//float64          pitch
		//float64          azimuth
		//float64          side_slip
		if (this->first_gps)
		{
			file << "time_s, time_ns, latitude, longitude, height, roll, pitch, azimuth,\n"; //headers for csv
			this->first_gps = false;
		}
		file << location->header.stamp.sec << "," <<
				location->header.stamp.nanosec << "," <<
				location->latitude << "," <<
				location->longitude << "," <<
				location->height << "," <<
				location->roll << "," <<
				location->pitch << "," <<
				location->azimuth << ",\n";		
	} else
	{
		std::cout << "Could not open file: " << this->settings.gpsfilename << "\n";
	}

	file.close();

	gpssave_mutex.unlock();
}

//Frame conversion helpers
// Eigen::Matrix3f rotx(const double roll)
// {
// 	Eigen::Matrix3f rot;
// 	//todo: make a rotation matrix
// 	return rot;
// }

// Eigen::Matrix3f roty(const double pitch)
// {
// 	Eigen::Matrix3f rot << std::cos(pitch), 0, std::sin(pitch),
						   
// 	return rot;
// }

// Eigen::Matrix3f rotz(const double yaw)
// {
// 	Eigen::Matrix3f rot << std::cos(yaw), -std::sin(yaw), 0,
// 						   std::sin(yaw), std::cos(yaw), 0,
// 						   0, 0, 1;
// 	return rot;
// }

void PointCloudMapper::e2a(double vout[3], Eigen::Matrix<double, 3, 1>& v)
{ 
    vout[0] = v(0) ; 
    vout[1] = v(1) ; 
    vout[2] = v(2) ;  
}

void PointCloudMapper::a2e( Eigen::Matrix<double, 3, 1>& vout, double v[3])
{ 
    vout(0) = v[0] ; 
    vout(1) = v[1] ; 
    vout(2) = v[2] ;  
}

Eigen::Matrix<double, 3, 1> PointCloudMapper::convertlla2ned(Eigen::Matrix<double, 3, 1>& lla, Eigen::Matrix<double, 3, 1>& ref_lla) 
{
    double lla_a[3], ref_lla_a[3], ned_a[3];
    Eigen::Matrix<double, 3, 1> ned ;  
    e2a(lla_a,lla) ; 
    e2a(ref_lla_a,ref_lla) ;
    this->frame_conversions->lla2ned(ned_a,lla_a,ref_lla_a) ; 
    a2e(ned, ned_a) ; 
    //std::cout << std::endl << "Convert LLA2NED: " << std::endl << ned_a[0] << std::endl << ned_a[1] << std::endl << ned_a[2] << std::endl << ned << std::endl << std::endl ;  
    return ned ; 
}

// -- end frame conversion helpers