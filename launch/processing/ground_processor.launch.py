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

	ground_processor_description = Node(
			package='pointcloud_utils',
			namespace='lidar_processing',
			executable='ground_processor',
			name='ground_processor',
			output='screen',
			parameters=[os.path.join(config_path, 'processing/ground_processor.yaml')]
		)

	return LaunchDescription([
		ground_processor_description,
	])