from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_navigator',
            executable='aruco_navigator.py',
            name='aruco_navigator',
            output='screen'
        )
    ])
