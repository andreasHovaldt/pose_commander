from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="pose_commander",
			executable="pose_publisher",
			output="screen"
		),
	])
