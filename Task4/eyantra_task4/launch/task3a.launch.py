from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    ebot_warehouse_dir = get_package_share_directory('eyantra_warehouse')
    ur_simulation_gazebo_dir = get_package_share_directory('ur_simulation_gazebo')

    # Include ebot_gazebo_task2b_launch.py from ebot_description package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ebot_warehouse_dir, 'launch', 'task2a.launch.py')
        )
    )

    robotic_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_simulation_gazebo_dir, 'launch', 'spawn_ur5_launch_moveit.launch.py')
        )
    )

    # Node for running payload.py
    tf_aruco_node = Node(
        package='ur5_control',
        executable='task1b',
        name='tf_aruco_node',
        output='screen'
    )

    # LaunchDescription that includes everything
    return LaunchDescription([
        gazebo_launch,
        tf_aruco_node,
        robotic_arm_launch
    ])
