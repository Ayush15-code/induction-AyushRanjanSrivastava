from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_path = get_package_share_directory('aruco_navigator')
    world_path = os.path.join(pkg_path, 'worlds', 'aruco_five_tags_world.sdf')
    model_path = os.path.join(pkg_path, 'models')

    # Set the model path so Gazebo can find your ArUco tags
    os.environ['GAZEBO_MODEL_PATH'] = model_path

    # Launch Gazebo with your custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn the TurtleBot3
    spawn_turtlebot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-database', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_turtlebot3,
    ])
