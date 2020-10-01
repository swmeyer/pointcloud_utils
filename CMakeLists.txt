cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  tf2_msgs
  tf2_sensor_msgs
  nav_msgs
  cv_bridge
  dynamic_reconfigure
  rosbag
)

generate_dynamic_reconfigure_options(
  cfg/pointcloud_utils.cfg
  #...
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
  LIBRARIES pointcloud_utils
  CATKIN_DEPENDS roscpp sensor_msgs tf2_msgs tf2_sensor_msgs nav_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#conversion
add_library(pointcloud_utils 
  src/conversion/pointcloud_grid_parser.cpp 
  src/io/pointcloud_saver.cpp 
  src/conversion/pointcloud_to_laserscan.cpp
  src/processing/intensity_filter.cpp)
target_link_libraries(pointcloud_utils ${catkin_LIBRARIES})

add_executable(pointcloud_grid_parser src/conversion/pointcloud_grid_parser_node.cpp src/conversion/pointcloud_grid_parser.cpp)
add_dependencies(pointcloud_grid_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
target_link_libraries(pointcloud_grid_parser ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan src/conversion/pointcloud_to_laserscan.cpp src/conversion/pointcloud_to_laserscan_node.cpp)
add_dependencies(pointcloud_to_laserscan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

#io
add_executable(pointcloud_saver src/io/pointcloud_saver_node.cpp src/io/pointcloud_saver.cpp)
add_dependencies(pointcloud_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_saver ${catkin_LIBRARIES})

add_executable(bag_parser src/io/bag_parser.cpp)
add_dependencies(bag_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
target_link_libraries(bag_parser ${catkin_LIBRARIES})

#oddball
add_executable(distance_marker_publisher src/oddball/distance_marker_publisher.cpp)
add_dependencies(distance_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_marker_publisher ${catkin_LIBRARIES})

#processing
add_executable(ground_bounce_counter src/processing/ground_error_detector.cpp)
add_dependencies(ground_bounce_counter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_bounce_counter ${catkin_LIBRARIES})

add_executable(intensity_filter src/processing/intensity_filter_node.cpp src/processing/intensity_filter.cpp)
add_dependencies(intensity_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(intensity_filter ${catkin_LIBRARIES})



#############
## INSTALL ##
#############

install (TARGETS pointcloud_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})