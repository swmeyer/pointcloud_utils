import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

	config_path = os.path.join(
		get_package_share_directory('pointcloud_utils'),
		'config'
		)

	cloud_saver_description = Node(
			package='pointcloud_utils',
			namespace='lidar_processing',
			executable='pointcloud_saver',
			name='pointcloud_saver',
			output='screen',
			parameters=[os.path.join(config_path, 'io/pointcloud_saver.yaml')]
		)

	return LaunchDescription([
		cloud_saver_description,
	])