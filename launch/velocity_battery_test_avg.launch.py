from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument("cmd_vel", default_value="0.5", description="Commanded linear velocity")

    cmd_time_arg = DeclareLaunchArgument(
        "cmd_time", default_value="3.0", description="Duration for each velocity command"
    )

    num_runs_arg = DeclareLaunchArgument(
        "num_runs", default_value="10", description="Number of forward/backward runs"
    )

    node = Node(
        package="asl_tb3_aiet",
        executable="velocity_battery_test_avg.py",
        name="velocity_battery_test_avg",
        output="screen",
        parameters=[
            {"cmd_vel": LaunchConfiguration("cmd_vel")},
            {"cmd_time": LaunchConfiguration("cmd_time")},
            {"num_runs": LaunchConfiguration("num_runs")},
        ],
    )

    return LaunchDescription([cmd_vel_arg, cmd_time_arg, num_runs_arg, node])
