# import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # config_path = os.path.join(
  #   get_package_share_directory('road_edge_detection'),
  #   'config'
  #   )

  cloud_reader_description = Node(
      package='pointcloud_utils',
      namespace='',
      executable='simple_cloud_reader',
      name='cloud_reader',
      output='screen',
      parameters=[
        {"cloud_topic": "/luminar_front_points"},
        {"use_sim_time": True}
      ]
    )

  return LaunchDescription([
    cloud_reader_description,
  ])