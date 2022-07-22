/** 
 * Author: Stephanie Meyer swmeyer16@gmail.com 1 Oct 2021
 * Brief: a class to subscribe to 3 point cloud messages and combine them into one message
 * File: pointcloud_combiner_node3.cpp
 */

// --------------------------
#include "rclcpp/rclcpp.hpp"

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

// #include <Eigen/Dense>

#include "pointcloud_utils/pointcloud_utils.hpp"
#include "pointcloud_utils/pointcloud_utils_impl.hpp"
#include "pointcloud_utils/io/pointcloud_saver.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <unistd.h>     //required for usleep()

// --------------------------

class PointCloudCombinerNode : public rclcpp::Node
{
	public:
		PointCloudCombinerNode() :
			Node("point_cloud_combiner_node"), 
			has_new_cloud1(false),
			has_new_cloud2(false),
			has_new_cloud3(false),
			finished_received(false)
		{
			//main content
			
			//declare params
			this->declare_parameter<std::string>("cloud_topic1", "/cloud1");
			this->declare_parameter<std::string>("cloud_topic2", "/cloud2");
			this->declare_parameter<std::string>("cloud_topic3", "/cloud3");
			this->declare_parameter<std::string>("target_frame", "base_link");
			this->declare_parameter<std::string>("cloud_out_topic", "/cloud");

			this->declare_parameter<bool>("use_luminar_pointstruct", true);
			this->declare_parameter<double>("rate", 10);
			this->declare_parameter<bool>("use_current_time", false);
			this->declare_parameter<bool>("wait_for_sync", false);
			this->declare_parameter<double>("old_cloud_timeout", 0.25);

			this->declare_parameter<int>("max_num_points", -1);
			this->declare_parameter<bool>("invert_point_crop", false);

			this->declare_parameter<bool>("drop_old_clouds", false);
			this->declare_parameter<bool>("read_from_bag", false);
            this->declare_parameter<std::string>("bagfile", "../rosbag2_test_data");
			this->declare_parameter<bool>("wait_for_finish_msg", false);
			this->declare_parameter<std::string>("finish_msg_topic", "/walls_finished");
			this->declare_parameter<float>("bag_read_wait", 0.25);
			this->declare_parameter<bool>("save_to_file", false);

			this->declare_parameter<bool>("invert_tf", false);

			this->declare_parameter<double>("x_trans_1", 0.0);
			this->declare_parameter<double>("y_trans_1", 0.0);
			this->declare_parameter<double>("z_trans_1", 0.0);
			this->declare_parameter<double>("roll_1", 0.0);
			this->declare_parameter<double>("pitch_1", 0.0);
			this->declare_parameter<double>("yaw_1", 0.0);

			this->declare_parameter<double>("x_trans_2", 0.0);
			this->declare_parameter<double>("y_trans_2", 0.0);
			this->declare_parameter<double>("z_trans_2", 0.0);
			this->declare_parameter<double>("roll_2", 0.0);
			this->declare_parameter<double>("pitch_2", 0.0);
			this->declare_parameter<double>("yaw_2", 0.0);

			this->declare_parameter<double>("x_trans_3", 0.0);
			this->declare_parameter<double>("y_trans_3", 0.0);
			this->declare_parameter<double>("z_trans_3", 0.0);
			this->declare_parameter<double>("roll_3", 0.0);
			this->declare_parameter<double>("pitch_3", 0.0);
			this->declare_parameter<double>("yaw_3", 0.0);


			//get params
			std::string cloud_out_topic, finish_msg_topic;
			double rate;

			this->get_parameter<std::string>("cloud_topic1", this->cloud_topic1);
			this->get_parameter<std::string>("cloud_topic2", this->cloud_topic2);
			this->get_parameter<std::string>("cloud_topic3", this->cloud_topic3);
			this->get_parameter<std::string>("target_frame", this->target_frame);
			this->get_parameter<std::string>("cloud_out_topic", cloud_out_topic);

			this->get_parameter<bool>("use_luminar_pointstruct", this->use_luminar_pointstruct);
			this->get_parameter<double>("rate", rate);
			this->get_parameter<bool>("use_current_time", 	this->use_current_time);
			this->get_parameter<bool>("wait_for_sync", 		this->wait_for_sync);
			this->get_parameter<double>("old_cloud_timeout", this->old_cloud_timeout);

			this->get_parameter<int>("max_num_points", 		this->max_num_points);
			this->get_parameter<bool>("invert_point_crop", 	this->invert_point_crop);

			this->get_parameter<bool>("drop_old_clouds", 	this->drop_old_clouds);
			this->get_parameter<bool>("read_from_bag", 		this->read_from_bag);
            this->get_parameter<std::string>("bagfile", 	this->bagfile);
			this->get_parameter<bool>("wait_for_finish_msg", this->wait_for_finish_msg);
			this->get_parameter<std::string>("finish_msg_topic", finish_msg_topic);
			this->get_parameter<float>("bag_read_wait", 	this->bag_read_wait);
			this->get_parameter<bool>("save_to_file", 		this->save_to_file);

			this->get_parameter<bool>("invert_tf", 			this->invert_tf);

			this->get_parameter<double>("x_trans_1", tf1.x);
			this->get_parameter<double>("y_trans_1", tf1.y);
			this->get_parameter<double>("z_trans_1", tf1.z);
			this->get_parameter<double>("roll_1", 	tf1.roll);
			this->get_parameter<double>("pitch_1", 	tf1.pitch);
			this->get_parameter<double>("yaw_1", 	tf1.yaw);

			this->get_parameter<double>("x_trans_2", tf2.x);
			this->get_parameter<double>("y_trans_2", tf2.y);
			this->get_parameter<double>("z_trans_2", tf2.z);
			this->get_parameter<double>("roll_2", 	tf2.roll);
			this->get_parameter<double>("pitch_2", 	tf2.pitch);
			this->get_parameter<double>("yaw_2", 	tf2.yaw);

			this->get_parameter<double>("x_trans_3", tf3.x);
			this->get_parameter<double>("y_trans_3", tf3.y);
			this->get_parameter<double>("z_trans_3", tf3.z);
			this->get_parameter<double>("roll_3", 	tf3.roll);
			this->get_parameter<double>("pitch_3", 	tf3.pitch);
			this->get_parameter<double>("yaw_3", 	tf3.yaw);

			if (this->save_to_file)
			{
				this->pt_cloud_saver = new pointcloud_utils::PointCloudSaver("points", ".csv");
			}

			if (this->wait_for_finish_msg)
			{
				this->finished_msg_sub = this->create_subscription<std_msgs::msg::Bool>(finish_msg_topic, 1, std::bind(&PointCloudCombinerNode::finishMsgCallback, this, std::placeholders::_1));
			}

			this->cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic, rclcpp::SensorDataQoS());

			RCLCPP_INFO(this->get_logger(), "ms delay: %d", (int) ((1/rate) * 1000));

			if (this->read_from_bag)
			{

  				this->bagread_thread = new std::thread(std::bind(&PointCloudCombinerNode::readFromBag, this));
				bagread_thread->detach();
			} else
			{
				this->cloud_sub1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->cloud_topic1, rclcpp::SensorDataQoS(), std::bind(&PointCloudCombinerNode::pointCloudCallback1, this, std::placeholders::_1));
				this->cloud_sub2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->cloud_topic2, rclcpp::SensorDataQoS(), std::bind(&PointCloudCombinerNode::pointCloudCallback2, this, std::placeholders::_1));
				this->cloud_sub3 = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->cloud_topic3, rclcpp::SensorDataQoS(), std::bind(&PointCloudCombinerNode::pointCloudCallback3, this, std::placeholders::_1));
			}

			timer = this->create_wall_timer(std::chrono::milliseconds( (int) ((1/rate) * 1000) ), std::bind(&PointCloudCombinerNode::timerCallback, this));
		}

		~PointCloudCombinerNode()
		{
			this->pt_cloud_saver->saveTimesToFile();
			delete this->pt_cloud_saver;
			
			this->bagread_thread->join();
			delete this->bagread_thread;

		}

	private:
		//Constants
		const int S_TO_MS = 1000000;

		//Variables
		bool use_luminar_pointstruct; //if true, use luminar pointstruct. otherwise, use basic pointstruct
		bool use_current_time; //if true, use time now in headers. otherwise, use sensor time
		bool wait_for_sync; // if true, wait for 3 new clouds before publishing. otherwise, repeat clouds as necessary
		bool invert_tf; //if true, invert the given tf's, otherwise, use them directly
		std::string target_frame; //frame to report the final cloud in
		double old_cloud_timeout; //[s] amount of time to maintain old clouds

		std::thread* bagread_thread;

		std::string cloud_topic1, cloud_topic2, cloud_topic3;
		std::string bagfile;

		int max_num_points;
		bool invert_point_crop;
		bool drop_old_clouds; 
		bool read_from_bag;
		bool wait_for_finish_msg;
		float bag_read_wait;
		bool save_to_file;

		sensor_msgs::msg::PointCloud2 cloud1;
		sensor_msgs::msg::PointCloud2 cloud2;
		sensor_msgs::msg::PointCloud2 cloud3;

		rclcpp::Time cloud1_time = rclcpp::Clock().now();
		rclcpp::Time cloud2_time = rclcpp::Clock().now();
		rclcpp::Time cloud3_time = rclcpp::Clock().now();

		pointcloud_utils::Transform tf1;
		pointcloud_utils::Transform tf2;
		pointcloud_utils::Transform tf3;

		bool has_new_cloud1;
		bool has_new_cloud2;
		bool has_new_cloud3;
		bool finished_received;

		std::mutex cloud1_mutex;
		std::mutex cloud2_mutex;
		std::mutex cloud3_mutex;

		std_msgs::msg::Header last_header;

		
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_msg_sub;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub1;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub2;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub3;


    	rclcpp::TimerBase::SharedPtr timer;

		pointcloud_utils::PointCloudSaver *pt_cloud_saver;


		//Methods

		bool processMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_msg, const std::string& topic_type, sensor_msgs::msg::PointCloud2& return_msg)
        {
    	
    		if (topic_type != "sensor_msgs/msg/PointCloud2")
    		{
    			return false;
    		}

            // deserialization and conversion to ros message
            auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
            ros_message->time_stamp = 0;
            ros_message->message = nullptr;
            ros_message->allocator = rcutils_get_default_allocator();

            rosbag2_cpp::SerializationFormatConverterFactory factory;
            std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer;
            cdr_deserializer = factory.load_deserializer("cdr");

            const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>();
            sensor_msgs::msg::PointCloud2 msg;
            ros_message->message = &msg;
            cdr_deserializer->deserialize(serialized_msg, type_support, ros_message);

            return_msg = msg;

            return true;
        }

		void readFromBag()
		{
        	rosbag2_cpp::readers::SequentialReader* reader = new rosbag2_cpp::readers::SequentialReader();
        	rosbag2_cpp::StorageOptions storage_options{};
			rosbag2_cpp::ConverterOptions converter_options{};

            storage_options.uri = this->bagfile;
            storage_options.storage_id = "sqlite3";

            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";

            reader->open(storage_options, converter_options);            
            std::vector<rosbag2_storage::TopicMetadata> topics = reader->get_all_topics_and_types();

            // metadata
            std::map<std::string, std::string> topics_map;
            for (rosbag2_storage::TopicMetadata topic:topics)
            {
                topics_map.insert(std::pair<std::string, std::string>(topic.name, topic.type));
            }

            // read and deserialize "serialized data"
            while (reader->has_next())
            {
                // serialized data
                std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message = reader->read_next();

                std::string topic_type = topics_map[serialized_message->topic_name];
                // std::cout << "Topic: " << topic_type << "\n";
                if (topic_type != "sensor_msgs/msg/PointCloud2")
                {
                	continue;
                }

                bool has_all_clouds = false;

                std::string topic_name = serialized_message->topic_name;
                sensor_msgs::msg::PointCloud2 msg;
                if (topic_name == this->cloud_topic1)
                {
                	if (!this->processMessage(serialized_message, topic_type, msg))
                	{
                		continue;
                	}
                	std::lock_guard<std::mutex> guard(this->cloud1_mutex);
					this->cloud1 = msg;
					this->last_header = msg.header;
					this->cloud1_time = rclcpp::Clock().now();
					this->has_new_cloud1 = true;
					has_all_clouds = (this->has_new_cloud1 && this->has_new_cloud2 && this->has_new_cloud3);
					// RCLCPP_INFO(this->get_logger(), "Found cloud1");
                } else if (topic_name == this->cloud_topic2)
                {
                	if (!this->processMessage(serialized_message, topic_type, msg))
                	{
                		continue;
                	}
                	std::lock_guard<std::mutex> guard(this->cloud1_mutex);
					this->cloud2 = msg;
					this->last_header = msg.header;
					this->cloud2_time = rclcpp::Clock().now();
					this->has_new_cloud2 = true;
					has_all_clouds = (this->has_new_cloud1 && this->has_new_cloud2 && this->has_new_cloud3);
					// RCLCPP_INFO(this->get_logger(), "Found cloud2");
                } else if (topic_name == this->cloud_topic3)
                {
                	if (!this->processMessage(serialized_message, topic_type, msg))
                	{
                		continue;
                	}
                	std::lock_guard<std::mutex> guard(this->cloud1_mutex);
					this->cloud3 = msg;
					this->last_header = msg.header;
					this->cloud3_time = rclcpp::Clock().now();
					this->has_new_cloud3 = true;
					has_all_clouds = (this->has_new_cloud1 && this->has_new_cloud2 && this->has_new_cloud3);
					// RCLCPP_INFO(this->get_logger(), "Found cloud3");
                }

                if (this->wait_for_finish_msg && has_all_clouds)
                {
                	RCLCPP_INFO(this->get_logger(), "waiting for finish process");
                	while(!this->finished_received && rclcpp::ok())
                	{
                		usleep(0.1 * S_TO_MS);
                	}
                	RCLCPP_INFO(this->get_logger(), "received process finished message!");
                	this->finished_received = false;
                } else if (has_all_clouds)
                {
     				//wait a bit before trying to do another one
					usleep(this->bag_read_wait * S_TO_MS);
                }

                
            }
            RCLCPP_INFO(this->get_logger(), "Finished reading bag");

			// this->pt_cloud_saver->saveTimesToFile();
		}

		void timerCallback()
		{

			// RCLCPP_INFO(this->get_logger(), "timer called");

			std::unique_lock<std::mutex> lock1(this->cloud1_mutex);
			std::unique_lock<std::mutex> lock2(this->cloud2_mutex);
			std::unique_lock<std::mutex> lock3(this->cloud3_mutex);

			if (!has_new_cloud1 && !has_new_cloud2 && !has_new_cloud3)
			{
				// RCLCPP_INFO(this->get_logger(), "waiting for more clouds to come in");
				return;
			}
			if (wait_for_sync)
			{
				if (!this->has_new_cloud1 || !this->has_new_cloud2 || !this->has_new_cloud3)
				{
					// RCLCPP_INFO(this->get_logger(), "waiting for more clouds to come in");
					return; //if we haven't recieved 3 new clouds, wait a cycle (rudamentary synchronization. we'll see how it does)
				} 
			}

			this->has_new_cloud1 = false;
			this->has_new_cloud2 = false;
			this->has_new_cloud3 = false;

			RCLCPP_INFO(this->get_logger(), "Processing cloud");

			sensor_msgs::msg::PointCloud2 pc2_cloud1 = this->cloud1;
			sensor_msgs::msg::PointCloud2 pc2_cloud2 = this->cloud2;
			sensor_msgs::msg::PointCloud2 pc2_cloud3 = this->cloud3;
			std_msgs::msg::Header header  = this->last_header;
			rclcpp::Time pc2_cloud1_time = this->cloud1_time;
			rclcpp::Time pc2_cloud2_time = this->cloud2_time;
			rclcpp::Time pc2_cloud3_time = this->cloud3_time;

			rclcpp::Time current_time = rclcpp::Clock().now();

			// rclcpp::Time current_time = rclcpp::Time(1, 0);

			lock1.unlock();
			lock2.unlock();
			lock3.unlock();

			rclcpp::Duration cloud1_diff = current_time - pc2_cloud1_time;
			rclcpp::Duration cloud2_diff = current_time - pc2_cloud2_time;
			rclcpp::Duration cloud3_diff = current_time - pc2_cloud3_time;

			double cloud1_diff_ = (static_cast<double>(cloud1_diff.seconds() + cloud1_diff.nanoseconds() * 1e-9));
			double cloud2_diff_ = (static_cast<double>(cloud2_diff.seconds() + cloud2_diff.nanoseconds() * 1e-9));
			double cloud3_diff_ = (static_cast<double>(cloud3_diff.seconds() + cloud3_diff.nanoseconds() * 1e-9));


			// RCLCPP_INFO( this->get_logger(), "cloud 1 time diff: %f ", cloud1_diff_ );
			// RCLCPP_INFO( this->get_logger(), "cloud 2 time diff: %f ", cloud2_diff_ );
			// RCLCPP_INFO( this->get_logger(), "cloud 3 time diff: %f ", cloud3_diff_ );


			if (this->drop_old_clouds)
			{
				if (!pointcloud_utils::inTolerance(cloud1_diff_, 0, this->old_cloud_timeout) )
				{
					RCLCPP_INFO(this->get_logger(), "Dropping old cloud 1");
					pc2_cloud1.data.clear();
					pc2_cloud1.width = 0;
					pc2_cloud1.row_step = 0;
				}
	
				if (!pointcloud_utils::inTolerance(cloud2_diff_, 0, this->old_cloud_timeout) )
				{
					RCLCPP_INFO(this->get_logger(), "Dropping old cloud 2");
					pc2_cloud2.data.clear();
					pc2_cloud2.width = 0;
					pc2_cloud2.row_step = 0;
				}
	
				if (!pointcloud_utils::inTolerance(cloud3_diff_, 0, this->old_cloud_timeout) )
				{
					RCLCPP_INFO(this->get_logger(), "Dropping old cloud 3");
					pc2_cloud3.data.clear();
					pc2_cloud3.width = 0;
					pc2_cloud3.row_step = 0;
				}
			}

			// RCLCPP_INFO(this->get_logger(), "here1");


			if (!pc2_cloud1.data.size() && !pc2_cloud2.data.size() && !pc2_cloud3.data.size())
			{
				RCLCPP_INFO(this->get_logger(), "none or empty clouds receieved");
				return;
			}


			if (this->use_current_time)
			{
				header.stamp = this->now();
			}

			// std::cout << "last header time: " << this->last_header.stamp.sec << "\n";

			sensor_msgs::msg::PointCloud2 combined_cloud;			
			combined_cloud.header = header;
			combined_cloud.header.frame_id = this->target_frame;

			// RCLCPP_INFO(this->get_logger(), "here2");
			if (use_luminar_pointstruct)
			{				
				// Convert all 3 current clouds to luminar pointstructs, transforming as necessary
				std::vector<pointcloud_utils::luminarPointstruct> cloud1;
				std::vector<pointcloud_utils::luminarPointstruct> cloud2;
				std::vector<pointcloud_utils::luminarPointstruct> cloud3;
				std::vector<pointcloud_utils::luminarPointstruct> cloud_final;

				// pointcloud_utils::convertFromPointCloud2(pc2_cloud1, cloud1);
				// pointcloud_utils::convertFromPointCloud2(pc2_cloud2, cloud2);
				// pointcloud_utils::convertFromPointCloud2(pc2_cloud3, cloud3);

				pointcloud_utils::convertFromPointCloud2(pc2_cloud1, cloud1, this->max_num_points, this->invert_point_crop);
				pointcloud_utils::convertFromPointCloud2(pc2_cloud2, cloud2, this->max_num_points, this->invert_point_crop);
				pointcloud_utils::convertFromPointCloud2(pc2_cloud3, cloud3, this->max_num_points, this->invert_point_crop);


				// RCLCPP_INFO(this->get_logger(), "here3");
				
				pointcloud_utils::transformCloud(cloud1, this->tf1, this->invert_tf);
				pointcloud_utils::transformCloud(cloud2, this->tf2, this->invert_tf);
				pointcloud_utils::transformCloud(cloud3, this->tf3, this->invert_tf);

				// RCLCPP_INFO(this->get_logger(), "here4");

				uint num_pts = cloud1.size() + cloud2.size() + cloud3.size();

				cloud_final.reserve( num_pts); // preallocate memory
				cloud_final.insert( cloud_final.end(), cloud1.begin(), cloud1.end() );
				cloud_final.insert( cloud_final.end(), cloud2.begin(), cloud2.end() );
				cloud_final.insert( cloud_final.end(), cloud3.begin(), cloud3.end() );

				// RCLCPP_INFO(this->get_logger(), "here5");
				std::vector<sensor_msgs::msg::PointField> fields;
				double point_step;

				if (pc2_cloud1.data.size())
				{
					fields = pc2_cloud1.fields;
					point_step = pc2_cloud1.point_step;
				} else if (pc2_cloud2.data.size())
				{
					fields = pc2_cloud2.fields;
					point_step = pc2_cloud2.point_step;
				} else if (pc2_cloud3.data.size())
				{
					fields = pc2_cloud3.fields;
					point_step = pc2_cloud3.point_step;
				} else
				{
					std::cout << "Warning: Only empty point data has been received.\n";

					fields = pc2_cloud1.fields;
					point_step = pc2_cloud1.point_step;
				}

				combined_cloud.fields = fields;
				combined_cloud.point_step = point_step;
				combined_cloud.width = num_pts;
				combined_cloud.height = 1;
				combined_cloud.row_step = combined_cloud.point_step * combined_cloud.width;
				combined_cloud.data.resize(combined_cloud.row_step);
				memcpy(&(combined_cloud.data[0]), &(cloud_final[0]), combined_cloud.point_step * cloud_final.size());
				// memcpy(&(combined_cloud.data[0]), &(cloud1[0]), combined_cloud.point_step * cloud1.size());
				// memcpy(&(combined_cloud.data[combined_cloud.point_step * cloud1.size()]), &(cloud2[0]), combined_cloud.point_step * cloud2.size());
				// memcpy(&(combined_cloud.data[combined_cloud.point_step * cloud2.size()]), &(cloud3[0]), combined_cloud.point_step * cloud3.size());


			} else
			{
				// Convert all 3 current clouds to pointstructs, transforming as necessary
				std::vector<pointcloud_utils::pointstruct> cloud1;
				std::vector<pointcloud_utils::pointstruct> cloud2;
				std::vector<pointcloud_utils::pointstruct> cloud3;
				std::vector<pointcloud_utils::pointstruct> cloud_final;

				pointcloud_utils::convertFromPointCloud2(pc2_cloud1, cloud1);
				pointcloud_utils::convertFromPointCloud2(pc2_cloud2, cloud2);
				pointcloud_utils::convertFromPointCloud2(pc2_cloud3, cloud3);

				pointcloud_utils::transformCloud(cloud1, this->tf1);
				pointcloud_utils::transformCloud(cloud2, this->tf2);
				pointcloud_utils::transformCloud(cloud3, this->tf3);


				cloud_final.reserve( cloud1.size() + cloud2.size() + cloud3.size()); // preallocate memory
				cloud_final.insert( cloud_final.end(), cloud1.begin(), cloud1.end() );
				cloud_final.insert( cloud_final.end(), cloud2.begin(), cloud2.end() );
				cloud_final.insert( cloud_final.end(), cloud3.begin(), cloud3.end() );

				combined_cloud.fields = pc2_cloud1.fields;
				combined_cloud.point_step = pc2_cloud1.point_step;
				combined_cloud.width = cloud_final.size();
				combined_cloud.height = 1;
				combined_cloud.row_step = combined_cloud.point_step * combined_cloud.width;
				combined_cloud.data.resize(combined_cloud.row_step);
				memcpy(&(combined_cloud.data[0]), &(cloud_final[0]), combined_cloud.row_step);
			}

			// std::cout << "combining clouds of data size: " << pc2_cloud1.data.size() << ", " << pc2_cloud2.data.size() << ", " << pc2_cloud3.data.size() << "\n";
			// std::cout << "with widths: " << pc2_cloud1.width << ", " << pc2_cloud2.width << ", " << pc2_cloud3.width << "\n";


			// RCLCPP_INFO(this->get_logger(), "here6");

			// std::cout << "about to publish cloud of size " << combined_cloud.data.size() << " bytes\n";
			if (combined_cloud.data.size() != 0)
			{	
				RCLCPP_INFO(this->get_logger(), "Publishing nonempty cloud");
				this->cloud_pub->publish(combined_cloud);
				if (this->save_to_file)
				{
					this->pt_cloud_saver->setCurrentCloud(combined_cloud);
				}
			}

		}

		void finishMsgCallback(const std_msgs::msg::Bool::SharedPtr msg )
		{
			this->finished_received = msg->data;
		}
		
		
		/**
		 * @Function 	pointCloudCallback1
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages.
		 */
		void pointCloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			std::lock_guard<std::mutex> guard(this->cloud1_mutex);
			this->cloud1 = *msg;
			this->last_header = msg->header;
			this->cloud1_time = rclcpp::Clock().now();
			this->has_new_cloud1 = true;
		}

		/**
		 * @Function 	pointCloudCallback2
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages.
		 */
		void pointCloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			std::lock_guard<std::mutex> guard(this->cloud2_mutex);
			this->cloud2 = *msg;
			this->last_header = msg->header;
			this->cloud2_time = rclcpp::Clock().now();
			this->has_new_cloud2 = true;
		}

		/**
		 * @Function 	pointCloudCallback3
		 * @Param 		msg - incoming data message
		 * @Return 		void
		 * @Brief 		Reacts to incoming pointcloud messages.
		 */
		void pointCloudCallback3(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
		{
			std::lock_guard<std::mutex> guard(this->cloud3_mutex);
			this->cloud3 = *msg;
			this->last_header = msg->header;
			this->cloud3_time = rclcpp::Clock().now();
			this->has_new_cloud3 = true;
		}
		
};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCombinerNode>());
    rclcpp::shutdown();
	return(0);
}
