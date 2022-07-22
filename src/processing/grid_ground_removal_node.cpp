/**
 * Ground removal using statistical analysis of height data from an occupancy grid
 *  
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>
#include <unordered_map>
#include <numeric>
#include <algorithm>
#include <math.h>

#include <iostream>

#include <omp.h>

#include "pointcloud_utils/pointcloud_utils.hpp"
#include "pointcloud_utils/pointcloud_utils_impl.hpp"

using sensor_msgs::msg::PointCloud2;

class GridGroundRemovalNode : public rclcpp::Node
{
    public:
        GridGroundRemovalNode() : Node("grid_ground_removal_node")
        {
            RCLCPP_INFO(this->get_logger(), "Grid ground processor init");

            // --- ROS parameters ---
            // IO params
            this->declare_parameter<std::string>("input_cloud_topic", "/luminar_points");
            this->declare_parameter<std::string>("output_nonground_topic", "/lidar/nonground_points");
            this->declare_parameter<bool>("use_lumin2", false);

            // ROI params
            this->declare_parameter<float>("x_min", -75.0);
            this->declare_parameter<float>("x_max", 75.0);
            this->declare_parameter<float>("y_min", -25.0);
            this->declare_parameter<float>("y_max", 25.0);
            this->declare_parameter<float>("z_min", -0.5);
            this->declare_parameter<float>("z_max", 2.5);

            this->declare_parameter<float>("ego_x_min", -3.0);
            this->declare_parameter<float>("ego_x_max", 3.0);
            this->declare_parameter<float>("ego_y_min", -2.0);
            this->declare_parameter<float>("ego_y_max", 2.0);

            // Grid params
            this->declare_parameter<float>("grid_resolution", 0.5);

            // Stats params
            this->declare_parameter<float>("height_variance_threshold", 0.02);
            this->declare_parameter<float>("min_height_threshold", 0.0);
            this->declare_parameter<int>("min_points_threshold", 5);

            // --- Initialize data ---
            this->get_parameter<bool>("use_lumin2", this->use_lumin2);
            this->get_parameter<float>("x_min", this->_x_min);
            this->get_parameter<float>("x_max", this->_x_max);
            this->get_parameter<float>("y_min", this->_y_min);
            this->get_parameter<float>("y_max", this->_y_max);
            this->get_parameter<float>("z_min", this->_z_min);
            this->get_parameter<float>("z_max", this->_z_max);

            this->get_parameter<float>("ego_x_min", this->_ego_x_min);
            this->get_parameter<float>("ego_x_max", this->_ego_x_max);
            this->get_parameter<float>("ego_y_min", this->_ego_y_min);
            this->get_parameter<float>("ego_y_max", this->_ego_y_max);

            this->get_parameter<float>("grid_resolution", this->_grid_res);
            this->get_parameter<float>("height_variance_threshold", this->_h_var_thresh);
            this->get_parameter<float>("min_height_threshold", this->_h_min_thresh);
            this->get_parameter<int>("min_points_threshold", this->_min_pts_thresh);

            this->_n_cols = (this->_y_max - this->_y_min) / this->_grid_res;
            this->_n_rows = (this->_x_max - this->_x_min) / this->_grid_res;

            std::cout << "Grid size: (" << this->_n_cols << ", " << this->_n_rows << ")" << std::endl;

            this->_height_accum = std::unordered_map<uint64_t, std::vector<float>>();
            this->_index_accum = std::unordered_map<uint64_t, std::vector<uint64_t>>();

            std::string input_cloud_topic, output_nonground_topic;
            this->get_parameter<std::string>("input_cloud_topic", input_cloud_topic);
            this->_lidar_sub = this->create_subscription<PointCloud2>(
                input_cloud_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&GridGroundRemovalNode::_pointCloudCallBack, this, std::placeholders::_1)
            );

            this->get_parameter<std::string>("output_nonground_topic", output_nonground_topic);
            this->_nonground_pub = this->create_publisher<PointCloud2>(
                output_nonground_topic,
                rclcpp::SensorDataQoS()
            );


        }
    
    private:
        // Objects
        rclcpp::Publisher<PointCloud2>::SharedPtr _nonground_pub;
        
        rclcpp::Subscription<PointCloud2>::SharedPtr _lidar_sub;

        std::unordered_map<uint64_t, std::vector<float>> _height_accum;
        std::unordered_map<uint64_t, std::vector<uint64_t>> _index_accum;

        bool use_lumin2;

        // Data
        float _x_min, _x_max, _y_min, _y_max, _z_min, _z_max;
        float _ego_x_min, _ego_x_max, _ego_y_min, _ego_y_max;
        float _grid_res, _n_cols, _n_rows;
        float _h_var_thresh, _h_min_thresh; 
        int _min_pts_thresh;


        // Methods
        template <class T> void processCloud(std::vector<T>& cloud, const PointCloud2::SharedPtr msg)
        {
            pointcloud_utils::convertFromPointCloud2(*msg, cloud);

            // Build occupancy grid
            #pragma omp simd
            for(uint64_t idx = 0; idx < cloud.size(); ++idx)
            {
                // Load point
                const T pt = cloud.at(idx);
                const float x = pt.x;
                const float y = pt.y;
                const float z = pt.z;

                // Check if it's valid
                if (this->_isOutOfBounds(pt))
                {
                    continue;
                }
                
                // Find the proper bin and insert
                uint64_t bin = this->_sub2ind(x, y);

                this->_index_accum[bin].push_back(idx);
                this->_height_accum[bin].push_back(z);
            }
            
            std::vector<uint64_t> valid_idxs;
            for (auto it : this->_height_accum)
            {
                const uint64_t key = it.first;
                const std::vector<float> heights = it.second;

                if (heights.size() < 2)
                {
                    continue;
                }

                // Check if there's enough variance in this bin
                const float var = this->_sampleVariance(heights);
                const float h_max = *std::max_element(heights.begin(), heights.end());

                // if so -> store indeces!
                if (var > this->_h_var_thresh && 
                    h_max > this->_h_min_thresh &&
                    heights.size() >= this->_min_pts_thresh)
                {
                    std::vector<uint64_t> good_idxs = this->_index_accum[key];
                    for (int64_t idx : good_idxs)
                    {
                        valid_idxs.push_back(idx);
                    }
                }
            }
            
            std::vector<T> out_cloud;
            out_cloud.reserve(valid_idxs.size());

            for (uint64_t idx : valid_idxs)
            {
                    auto pt = cloud.at(idx);
                    out_cloud.push_back(pt);
            }

            // Publish
            PointCloud2 out_msg;
            out_msg.header = msg->header;
            out_msg.fields = msg->fields;
            out_msg.point_step = msg->point_step;
            out_msg.height = 1;
            out_msg.width = out_cloud.size();
            out_msg.row_step = out_msg.point_step * out_cloud.size();
            out_msg.data.resize(out_msg.row_step);
            memcpy(&(out_msg.data[0]), &(out_cloud[0]), out_msg.row_step);

            this->_nonground_pub->publish(out_msg);
        }

        void _pointCloudCallBack(const PointCloud2::SharedPtr msg)
        {
            this->_height_accum.clear();
            this->_index_accum.clear();

            // Read into struct
            if (this->use_lumin2)
            {
                // std::cout << "Using lumin2 pointstruct\n";
                std::vector<pointcloud_utils::luminarPointstruct2> cloud;
                processCloud(cloud, msg);
            } else
            {
                std::vector<pointcloud_utils::luminarPointstruct> cloud;
                processCloud(cloud, msg);
            }
        }

        #pragma omp declare simd
        uint64_t _sub2ind(const float x, const float y)
        {
            // x \in [x_min, x_max]
            // y \in [y_min, y_max]

            // u \in [0, (x_max - x_min)/res]
            // v \in [0, (y_max - y_min)/res]

            uint64_t u = floor((x - this->_x_min) / this->_grid_res);
            uint64_t v = floor((y - this->_y_min) / this->_grid_res);

            return u * this->_n_cols + v;
        }

        #pragma omp declare simd
        template <class T> bool _isOutOfBounds(const T& pt)
        {
            const bool invalid_x = (pt.x < this->_x_min) || (pt.x > this->_x_max);
            const bool invalid_y = (pt.y < this->_y_min) || (pt.y > this->_y_max);
            const bool invalid_z = (pt.z < this->_z_min) || (pt.z > this->_z_max);

            const bool is_ego = (pt.x > this->_ego_x_min) && (pt.x < this->_ego_x_max) && 
                (pt.y > this->_ego_y_min) && (pt.y < this->_ego_y_max);
            
            return invalid_x || invalid_y || invalid_z || is_ego;
        }

        #pragma omp declare simd
        float _sampleVariance(const std::vector<float>& data)
        {
            const size_t sz = data.size();
            const float mean = std::accumulate(data.begin(), data.end(), 0.0) / sz;

            auto var_func = [&mean, &sz](float accum, const float& val) {
                return accum +( (val-mean)*(val-mean) / (sz - 1) );
            };

            return std::accumulate(data.begin(), data.end(), 0.0, var_func);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridGroundRemovalNode>());
    rclcpp::shutdown();
    return(0);
}