#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("asl_tb3_aiet"),
                    "rviz",
                    "default.rviz",
                ]),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            executable="rqt_gui",
            package="rqt_gui",
            arguments=["--perspective-file", PathJoinSubstitution([
                FindPackageShare("asl_tb3_aiet"),
                "rqt",
                "state.perspective",
            ])],
            parameters=[{"use_sim_time": use_sim_time}]
        ),

        # relay RVIZ goal pose to some other channel
        Node(
            executable="rviz_goal_relay.py",
            package="asl_tb3_lib",
            parameters=[
                {"output_channel": "/cmd_nav"},
            ],
        ),

        # publish robot state from TF tree
        Node(
            executable="state_publisher.py",
            package="asl_tb3_lib",
        ),

        # student's navigatior node
        Node(
            executable="navigator.py",
            package="asl_tb3_aiet",
            parameters=[{"use_sim_time": use_sim_time}]
        ),

        # student's frontier exploration node
        Node(
            executable="frontier_explorer.py",
            package="asl_tb3_aiet",
            parameters=[{"use_sim_time": use_sim_time}]
        ),
    ])
