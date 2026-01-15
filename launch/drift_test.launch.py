from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument("cmd_vel", default_value="0.5", description="Commanded linear velocity")

    cmd_vel_ang_arg = DeclareLaunchArgument(
        "cmd_vel_ang", default_value="0.0", description="Commanded angular velocity"
    )

    cmd_time_arg = DeclareLaunchArgument(
        "cmd_time", default_value="3.0", description="Duration for each velocity command"
    )

    num_iterations_arg = DeclareLaunchArgument(
        "num_iterations", default_value="10", description="Number of forward/backward iterations"
    )

    node = Node(
        package="asl_tb3_aiet",
        executable="drift_test.py",
        name="drift_test",
        output="screen",
        parameters=[
            {"cmd_vel": LaunchConfiguration("cmd_vel")},
            {"cmd_vel_ang": LaunchConfiguration("cmd_vel_ang")},
            {"cmd_time": LaunchConfiguration("cmd_time")},
            {"num_iterations": LaunchConfiguration("num_iterations")},
        ],
    )

    return LaunchDescription([cmd_vel_arg, cmd_vel_ang_arg, cmd_time_arg, num_iterations_arg, node])
