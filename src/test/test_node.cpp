//test_node.cpp

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "pointcloud_utils/test/test.hpp"
#include "pointcloud_utils/test/test_impl.hpp"

class TestNode : public rclcpp::Node
{
	public:
		TestNode() : 
			Node("test_node"),
			place(0.0)
		{
			tester = new testClass<int>();

			marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/test_markers", 1);

			while (rclcpp::ok())
			{
				publishObjects();
				rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));

				this->place += 1.0;
			}
		}

		~TestNode()
		{

		}

	private:
		testClass<int>* tester;

		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

		double place;


		void publishObjects()
        {
        	geometry_msgs::msg::Point center;
        	center.x = 0;
        	center.y = place;
        	center.z = 0;

        	double height = 10.0;
        	double width = 10.0;
        	double depth = 10.0;

            //publish those boxes and points!

            //Bounding box marker
            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker box_marker;
            // box_marker.header = header;
            box_marker.ns = "boxes";
            box_marker.type = visualization_msgs::msg::Marker::LINE_STRIP; //Or LINE_STRIP?
            box_marker.scale.x = 0.1; //line thickness
            box_marker.scale.y = 0.1;
            box_marker.color.a = 1.0;
            box_marker.color.r = 0.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 0.0;

            //add endpoint pairs to the box_marker.points
            geometry_msgs::msg::Point pt1;
            geometry_msgs::msg::Point pt2;
            geometry_msgs::msg::Point pt3;
            geometry_msgs::msg::Point pt4;
            geometry_msgs::msg::Point pt5;
            geometry_msgs::msg::Point pt6;
            geometry_msgs::msg::Point pt7;
            geometry_msgs::msg::Point pt8;

            //Point cluster marker
            visualization_msgs::msg::Marker points_marker;
            points_marker = box_marker;
            points_marker.color.r = 1.0;
            points_marker.color.g = 0.0;
            points_marker.ns = "points";
            points_marker.type = visualization_msgs::msg::Marker::POINTS;

            geometry_msgs::msg::Point pt; //generic point


            //Populate

            for (uint i = 0; i < 2; i++)
        	{
        		box_marker.points.clear();
            	box_marker.id = i;

            	geometry_msgs::msg::Point temp = center;
            	temp.x += i;
            	temp.y += i;
            	temp.z += i;
	
            	pt1.x = temp.x + depth/2;
            	pt1.y = temp.y + width/2;
            	pt1.z = temp.z - height/2;
            	    
            	pt2 = pt1;
            	pt2.y = temp.y - width/2;
            	
            	pt3 = pt2;
            	pt3.x = temp.x - depth/2;
            	    
            	pt4 = pt3;
            	pt4.y = temp.y + width/2;
            	    
            	pt5 = pt1;
            	pt5.z = temp.z + height/2;
            	    
            	pt6 = pt2;
            	pt6.z = temp.z + height/2;
            	    
            	pt7 = pt3;
            	pt7.z = temp.z + height/2;
            	    
            	pt8 = pt4;
            	pt8.z = temp.z + height/2;  
	
            	//TODO: minimize number of points needed to draw box
            	box_marker.points.push_back(pt1);
            	box_marker.points.push_back(pt2);
            	box_marker.points.push_back(pt3);
            	box_marker.points.push_back(pt4);  
            	box_marker.points.push_back(pt1); 
	
            	box_marker.points.push_back(pt5);
            	box_marker.points.push_back(pt6);
            	box_marker.points.push_back(pt7);
            	box_marker.points.push_back(pt8); 
            	box_marker.points.push_back(pt5);
	
            	box_marker.points.push_back(pt2);
            	box_marker.points.push_back(pt6);
	
            	box_marker.points.push_back(pt3);
            	box_marker.points.push_back(pt7);
	
            	box_marker.points.push_back(pt4);
            	box_marker.points.push_back(pt8);
	
            	markers.markers.push_back(box_marker);
        			
        	}


            points_marker.points.clear();
            points_marker.id = 0;
            for (int i = 0; i < 10; i++)
            {

                pt.x = 0;
                pt.y = place;
                pt.z = i;

                points_marker.points.push_back(pt);
            }
            markers.markers.push_back(points_marker);

            marker_pub->publish(markers);
        }

};

int main(int argc, char* argv[])
{	
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
	return(0);
}