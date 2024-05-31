import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_detection',
            executable='apriltag_detection',
            name='apriltag_detection'
        ),
        Node(
            package='apriltag_detection',
            executable='tf_publisher',
            name='tf_publisher'
        )
    ])
