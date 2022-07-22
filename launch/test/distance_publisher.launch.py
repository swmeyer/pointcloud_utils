# import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # config_path = os.path.join(
  #   get_package_share_directory('road_edge_detection'),
  #   'config'
  #   )

  distance_pub_description = Node(
      package='pointcloud_utils',
      namespace='',
      executable='distance_marker_publisher',
      name='distance_marker_pub',
      output='screen',
      parameters=[
        {"publish_topic": "/distance_markers"},
        {"publish_frame": "luminar_front"},
        {"publish_rate:": 1},
        {"distance_interval": 10.0},
        {"number_of_markers": 50},
        {"marker_width": 20.0}
      ]
    )

  return LaunchDescription([
    distance_pub_description,
  ])