#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
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
            ),

            # state publisher for turtlebot
            Node(
                executable="state_publisher.py",
                package="asl_tb3_lib",
            ),
            
            # localization node
            Node(
                executable="localize_targets.py",
                package="asl_tb3_aiet",
            ),

            # navigatior node - listening on /cmd_nav
            Node(
                executable="navigator.py",
                package="asl_tb3_aiet",
                parameters=[{"use_sim_time": use_sim_time}]
            ),

            # simple nav controller
            Node(
                executable="simple_nav.py",
                package="asl_tb3_aiet",
            ),
        ]
    )
