import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'install/apriltag_detection/share/apriltag_detection/rviz/apriltag_rviz.rviz'],
            output='screen'
        )
    ])

