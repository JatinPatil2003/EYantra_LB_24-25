import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the robot description command to convert xacro to urdf
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "urdf",
                    "camera.urdf.xacro",  # Change this to your xacro file
                ]
            ),
        ]
    )
    
    # Wrap the output in ParameterValue, setting it as a string
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'realsense_camera',  # Name of the entity
            '-topic', 'robot_description',
            '-x', '1.6', '-y', '-2.4', '-z', '0.58', '-Y', '3.14'
        ],
        output='screen'
    )

    # Return LaunchDescription with nodes
    return LaunchDescription(
        [
            # joint_state_publisher_node,  # Publishes joint states
            robot_state_publisher_node,  # Publishes robot description
            spawn_camera,  # Spawns the camera in Gazebo
        ]
    )
