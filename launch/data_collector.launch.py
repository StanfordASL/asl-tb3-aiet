#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the data directory argument
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=[EnvironmentVariable('HOME'), '/driving_data'],
        description='Directory to store collected data'
    )

    # Create the data collector node
    data_collector_node = Node(
        package='asl_tb3_aiet',
        executable='data_collector.py',
        name='data_collector',
        output='screen',
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir')
        }]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_rviz = IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]
                ),
                launch_arguments={
                    "config": PathJoinSubstitution(
                        [
                            FindPackageShare("asl_tb3_aiet"),
                            "rviz",
                            "default.rviz",
                        ]
                    ),
                    "use_sim_time": use_sim_time,
                }.items(),
            )

    # Create and return the launch description
    return LaunchDescription([
        launch_rviz,
        data_dir_arg,
        data_collector_node
    ])