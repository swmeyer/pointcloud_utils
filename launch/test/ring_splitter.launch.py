# import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # config_path = os.path.join(
  #   get_package_share_directory('road_edge_detection'),
  #   'config'
  #   )

  ring_splitter_description = Node(
      package='pointcloud_utils',
      namespace='',
      executable='ring_splitter',
      name='ring_splitter',
      output='screen',
      parameters=[
        {"cloud_topic": "/luminar_front_points"},
        {"visualization_topic": "/ring_markers"},
        {"ring_group_size": 1},
        {"point_size": 0.5}
      ]
    )

  return LaunchDescription([
    ring_splitter_description,
  ])