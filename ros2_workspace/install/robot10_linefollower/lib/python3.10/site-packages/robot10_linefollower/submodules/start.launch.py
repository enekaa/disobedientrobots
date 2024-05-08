from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="ultrasonic-test",
			executable="line_tracker_node",
			name="line_tracker_node",
			output='screen',	
		),
	])
