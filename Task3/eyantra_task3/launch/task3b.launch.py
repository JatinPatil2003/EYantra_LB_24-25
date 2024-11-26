from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    ebot_description_dir = get_package_share_directory('ebot_description')
    ebot_nav2_dir = get_package_share_directory('ebot_nav2')

    # Include ebot_gazebo_task2b_launch.py from ebot_description package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ebot_description_dir, 'launch', 'ebot_gazebo_task2b_launch.py')
        )
    )

    naviagtion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ebot_nav2_dir, 'launch', 'navigation.launch.py')
        )
    )

    # Node for running payload.py
    payload_node = Node(
        package='ebot_description',
        executable='payload.py',
        name='payload_node',
        output='screen'
    )

    # Node for running ebot_docking_boilerplate.py
    docking_node = Node(
        package='ebot_docking',
        executable='ebot_docking_boilerplate.py',
        name='docking_node',
        output='screen'
    )

    # LaunchDescription that includes everything
    return LaunchDescription([
        gazebo_launch,
        payload_node,
        docking_node,
        naviagtion_launch
    ])
