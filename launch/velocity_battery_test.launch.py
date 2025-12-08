from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument("cmd_vel", default_value="0.5", description="Commanded linear velocity")

    cmd_time_arg = DeclareLaunchArgument(
        "cmd_time", default_value="3.0", description="Duration for each velocity command"
    )

    node = Node(
        package="asl_tb3_aiet",
        executable="velocity_battery_test.py",
        name="velocity_battery_test",
        output="screen",
        parameters=[{"cmd_vel": LaunchConfiguration("cmd_vel")}, {"cmd_time": LaunchConfiguration("cmd_time")}],
    )

    return LaunchDescription([cmd_vel_arg, cmd_time_arg, node])
