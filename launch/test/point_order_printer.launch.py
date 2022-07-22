# import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # config_path = os.path.join(
  #   get_package_share_directory('road_edge_detection'),
  #   'config'
  #   )

  point_order_printer_description = Node(
      package='pointcloud_utils',
      namespace='',
      executable='point_order_printer',
      name='point_order_printer',
      output='screen',
      parameters=[
        {"cloud_topic": "/luminar_front_points"},
        {"visualization_topic": "/point_markers"},
        {"fill_time": 0.05},
        {"point_size": 0.5},
        {"print_one_ring": False},
        {"ring_id": 20},
        {"const_depth": True},
        {"depth_value": 100.0}
      ]
    )

  return LaunchDescription([
    point_order_printer_description,
  ])