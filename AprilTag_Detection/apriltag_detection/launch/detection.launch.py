import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_detection',
            executable='apriltag_detection_V1',
            name='apriltag_detection_V1'
        ),
        Node(
            package='apriltag_detection',
            executable='tf_publisher_V1',
            name='tf_publisher_V1'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'install/apriltag_detection/share/apriltag_detection/rviz/apriltag_rviz.rviz'],
            output='screen'
        )
    ])

