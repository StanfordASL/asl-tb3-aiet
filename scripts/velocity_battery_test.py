#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import csv
import os
import time


class VelocityBatteryTest(Node):
    def __init__(self):
        super().__init__("velocity_battery_test")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("cmd_vel", 0.5)
        self.declare_parameter("cmd_vel_ang", 0.0)
        self.declare_parameter("cmd_time", 3.0)

        self.cmd_vel_value = float(self.get_parameter("cmd_vel").value)
        self.cmd_vel_ang_value = float(self.get_parameter("cmd_vel_ang").value)
        self.cmd_time_value = float(self.get_parameter("cmd_time").value)

        self.get_logger().info(f"Starting test with cmd_vel={self.cmd_vel_value}, cmd_time={self.cmd_time_value}")

        # ---------------------------
        # ROS interfaces
        # ---------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(BatteryState, "/battery_state", self.battery_callback, 10)

        # Buffers
        self.cmd_vel_msg = Twist()
        self.measured_vel = None
        self.battery_state = None
        self.initial_battery_value = None

        # ---------------------------
        # Create output file
        # ---------------------------
        self.results_dir = os.path.expanduser("~/asl_tb3_aiet/results")
        os.makedirs(self.results_dir, exist_ok=True)

        # File will be created once battery value is known
        self.csv_writer = None
        self.logfile = None

        # Timing
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    # ---------------------------
    # Callbacks
    # ---------------------------
    def odom_callback(self, msg):
        self.measured_vel = msg.twist.twist

    def battery_callback(self, msg):
        self.battery_state = msg
        if self.initial_battery_value is None:
            self.initial_battery_value = round(msg.percentage * 100, 1)  # percentage 0-1 → %
            self.setup_logfile()

    # ---------------------------
    # Setup logfile once battery is known
    # ---------------------------
    def setup_logfile(self):
        filename = f"vel_results_bat_{self.initial_battery_value}_vel_{self.cmd_vel_value}_t_{self.cmd_time_value}.csv"
        path = os.path.join(self.results_dir, filename)

        self.logfile = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.logfile)

        self.csv_writer.writerow(
            ["time", "cmd_vx", "cmd_wz", "meas_vx", "meas_wz", "battery_voltage", "battery_percentage"]
        )

        self.get_logger().info(f"Logging to: {path}")

    # ---------------------------
    # Control + Logging loop
    # ---------------------------
    def update(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        # Command schedule:
        #   +cmd_vel for first cmd_time seconds
        #   -cmd_vel for next cmd_time seconds
        #   then stop
        if t < self.cmd_time_value:
            self.cmd_vel_msg.linear.x = self.cmd_vel_value
            self.cmd_vel_msg.angular.z = self.cmd_vel_ang_value
        elif t < 2 * self.cmd_time_value:
            self.cmd_vel_msg.linear.x = -self.cmd_vel_value
            self.cmd_vel_msg.angular.z = -self.cmd_vel_ang_value
        else:
            self.cmd_vel_msg.linear.x = 0.0

        # self.cmd_vel_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_vel_msg)

        # Don't log until everything is available
        if self.csv_writer is None or self.measured_vel is None or self.battery_state is None:
            return

        self.csv_writer.writerow(
            [
                now.nanoseconds,
                self.cmd_vel_msg.linear.x,
                self.cmd_vel_msg.angular.z,
                self.measured_vel.linear.x,
                self.measured_vel.angular.z,
                self.battery_state.voltage,
                self.battery_state.percentage,
            ]
        )

        # End experiment 2 seconds after motion ends
        if t > (2 * self.cmd_time_value + 2.0):
            self.get_logger().info("Experiment complete. Shutting down.")
            self.cleanup_and_shutdown()

    def cleanup_and_shutdown(self):
        if self.logfile:
            self.logfile.close()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBatteryTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.cleanup_and_shutdown()


if __name__ == "__main__":
    main()
