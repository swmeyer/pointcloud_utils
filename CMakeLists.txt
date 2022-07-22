cmake_minimum_required(VERSION 3.5)
project(pointcloud_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(atr_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(dynamic_reconfigure)
# find_package(rosbag)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rosbag2_cpp REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# generate_dynamic_reconfigure_options(
#   cfg/pointcloud_utils.cfg
#   #...
# )

include_directories(
  include
  include/pointcloud_utils
)

# library 
set(HEADER_FILES 
    include/pointcloud_utils/pointcloud_utils.hpp
    include/pointcloud_utils/pointcloud_utils_impl.hpp
    include/pointcloud_utils/conversion/pointcloud_grid_parser.hpp
    include/pointcloud_utils/conversion/pointcloud_grid_parser_impl.hpp
    # include/pointcloud_utils/conversion/pointcloud_stamper.hpp
    # include/pointcloud_utils/pointcloud_to_laserscan.hpp
    # include/pointcloud_utils/pointcloud_transformer.hpp

    include/pointcloud_utils/io/pointcloud_saver.hpp

    # include/pointcloud_utils/processing/ground_processor.hpp
    # include/pointcloud_utils/processing/intensity_filter.hpp
    # include/pointcloud_utils/processing/plane_parser.hpp

)

set(SOURCE_FILES
  src/conversion/pointcloud_grid_parser.cpp 
  src/io/pointcloud_saver.cpp 
  # src/conversion/pointcloud_to_laserscan.cpp
  # src/conversion/pointcloud_transformer.cpp
  # src/processing/intensity_filter.cpp
  # src/processing/plane_parser.cpp 
  # src/processing/ground_processor.cpp 
)

add_library(pointcloud_utils ${SOURCE_FILES} ${HEADER_FILES} )
ament_target_dependencies(pointcloud_utils sensor_msgs Eigen3)

ament_export_targets(pointcloud_utils HAS_LIBRARY_TARGET)
# ament_export_dependencies(sensor_msgs)

#Executables

#conversion
add_executable(pointcloud_grid_parser src/conversion/pointcloud_grid_parser_node.cpp src/conversion/pointcloud_grid_parser.cpp)
ament_target_dependencies(pointcloud_grid_parser rclcpp std_msgs sensor_msgs nav_msgs tf2_sensor_msgs geometry_msgs)
# target_link_libraries(pointcloud_grid_parser pointcloud_utils)

add_executable(cloud_combiner src/conversion/pointcloud_combiner_node3.cpp src/io/pointcloud_saver.cpp)
ament_target_dependencies(cloud_combiner rclcpp std_msgs sensor_msgs Eigen3 rosbag2_cpp)
# target_link_libraries(cloud_combiner pointcloud_utils::pointcloud_utils)

#  add_executable(pointcloud_to_laserscan src/conversion/pointcloud_to_laserscan.cpp src/conversion/pointcloud_to_laserscan_node.cpp)
#  ament_target_dependencies(pointcloud_to_laserscan )
#  
#  add_executable(pointcloud_transformer src/conversion/pointcloud_transformer.cpp src/conversion/pointcloud_transformer_node.cpp)
#  ament_target_dependencies(pointcloud_transformer )
#  
 #io
add_executable(pointcloud_saver src/io/pointcloud_saver_node.cpp src/io/pointcloud_saver.cpp)
ament_target_dependencies(pointcloud_saver rclcpp sensor_msgs Eigen3)

add_executable(pointcloud_mapper src/io/pointcloud_mapper_node.cpp src/io/pointcloud_mapper.cpp src/io/frame_conversions.cpp)
ament_target_dependencies(pointcloud_mapper rclcpp sensor_msgs atr_msgs novatel_oem7_msgs Eigen3)

#  
#  # add_executable(bag_parser src/io/bag_parser.cpp)
#  # ament_target_dependencies(bag_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
#  
#processing
#  add_executable(intensity_filter src/processing/intensity_filter_node.cpp src/processing/intensity_filter.cpp)
#  ament_target_dependencies(intensity_filter )
#  
#  add_executable(plane_parser src/processing/plane_parser_node.cpp src/processing/plane_parser.cpp)
#  ament_target_dependencies(plane_parser )
#  
add_executable(ground_processor src/processing/ground_removal_and_alignment_node.cpp)
ament_target_dependencies(ground_processor rclcpp sensor_msgs visualization_msgs geometry_msgs Eigen3)

add_executable(grid_ground_processor src/processing/grid_ground_removal_node.cpp src/io/pointcloud_saver.cpp)
ament_target_dependencies(grid_ground_processor rclcpp rosbag2_cpp sensor_msgs Eigen3 OpenMP)

#test
add_executable(simple_cloud_reader src/test/cloud_reader.cpp)
ament_target_dependencies(simple_cloud_reader rclcpp sensor_msgs Eigen3)

add_executable(distance_marker_publisher src/test/distance_marker_publisher.cpp)
ament_target_dependencies(distance_marker_publisher rclcpp visualization_msgs)

add_executable(ring_splitter src/test/ring_splitter.cpp)
ament_target_dependencies(ring_splitter rclcpp sensor_msgs visualization_msgs Eigen3)

add_executable(point_order_printer src/test/point_order_printer.cpp)
ament_target_dependencies(point_order_printer rclcpp sensor_msgs visualization_msgs Eigen3)

add_executable(test_node src/test/test_node.cpp )
ament_target_dependencies(test_node rclcpp visualization_msgs)

#  
#  add_executable(plane_points_generator src/test/plane_points_generator.cpp)
#  ament_target_dependencies(plane_points_generator )
#  
#  add_executable(imu_to_euler src/test/imu_to_euler.cpp)
#  ament_target_dependencies(imu_to_euler )

#############
## INSTALL ##
#############

#executables

install(
  TARGETS 
    pointcloud_grid_parser 
    cloud_combiner
    #pointcloud_to_laserscan
    #pointcloud_transformer
    pointcloud_saver
    pointcloud_mapper
    #intensity_filter
    #plane_parser
    ground_processor
    simple_cloud_reader
    distance_marker_publisher
    ring_splitter
    point_order_printer
    test_node
    #plane_points_generator
    #imu_to_euler
    grid_ground_processor
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#library
install(
  DIRECTORY include/pointcloud_utils
  DESTINATION include
)

install(
  TARGETS pointcloud_utils
  EXPORT pointcloud_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()