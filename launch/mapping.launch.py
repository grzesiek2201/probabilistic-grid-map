from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapping',
            executable='map_tf',
            name='map_tf'
        ),
        Node(
            package='mapping',
            executable='robot_marker_publisher',
            name='robot_marker'
        ),
        Node(
            package='mapping',
            executable='mapping_gazebo',
            name='mapper'
        )
    ])