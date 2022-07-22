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

	cloud_combiner_description = Node(
			package='pointcloud_utils',
			namespace='lidar_processing',
			executable='cloud_combiner',
			name='cloud_combiner',
			output='screen',
			parameters=[os.path.join(config_path, 'conversion/cloud_combiner.yaml')]
		)

	return LaunchDescription([
		cloud_combiner_description,
	])