#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP_NAME = 'slam_1'  # change to the name of your own map here


def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('ebot_nav2'), 'rviz', 'nav2_default_view.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('ebot_nav2'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('ebot_nav2'), 'params', 'my_nav2.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='True',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='True',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('sim'),
                'params_file': nav2_config_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('sim')}]
        )
    ])