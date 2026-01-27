#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import os
import math


class DriftTest(Node):
    def __init__(self):
        super().__init__("drift_test")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("cmd_vel", 0.5)
        self.declare_parameter("cmd_vel_ang", 0.0)
        self.declare_parameter("cmd_time", 3.0)
        self.declare_parameter("num_iterations", 10)

        self.cmd_vel_value = float(self.get_parameter("cmd_vel").value)
        self.cmd_vel_ang_value = float(self.get_parameter("cmd_vel_ang").value)
        self.cmd_time_value = float(self.get_parameter("cmd_time").value)
        self.num_iterations = int(self.get_parameter("num_iterations").value)

        self.get_logger().info(
            f"Starting drift test with cmd_vel={self.cmd_vel_value} (type: {type(self.cmd_vel_value)}), cmd_vel_ang={self.cmd_vel_ang_value} (type: {type(self.cmd_vel_ang_value)}), cmd_time={self.cmd_time_value}, iterations={self.num_iterations}"
        )

        # ---------------------------
        # ROS interfaces
        # ---------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Buffers
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.linear.y = 0.0
        self.cmd_vel_msg.linear.z = 0.0
        self.cmd_vel_msg.angular.x = 0.0
        self.cmd_vel_msg.angular.y = 0.0
        self.cmd_vel_msg.angular.z = 0.0

        self.current_pose = None
        self.initial_pose = None

        # ---------------------------
        # Create output file
        # ---------------------------
        self.results_dir = os.path.expanduser("~/asl_tb3_aiet/results")
        os.makedirs(self.results_dir, exist_ok=True)

        filename = f"drift_test_vel_{self.cmd_vel_value}_ang_{self.cmd_vel_ang_value}_t_{self.cmd_time_value}_iter_{self.num_iterations}.csv"
        path = os.path.join(self.results_dir, filename)

        self.logfile = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.logfile)

        self.csv_writer.writerow(
            ["iteration", "cmd_vx", "cmd_wz", "pos_x", "pos_y", "longitudinal_drift", "lateral_drift"]
        )

        self.get_logger().info(f"Logging to: {path}")

        # Timing and state tracking
        self.start_time = None
        self.current_iteration = 0
        self.phase = "waiting_initial"  # waiting_initial, forward, backward, complete

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    # ---------------------------
    # Callbacks
    # ---------------------------
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # ---------------------------
    # Helper functions
    # ---------------------------
    def get_yaw_from_quaternion(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def log_drift(self, iteration, cmd_vx, cmd_wz):
        """Log drift from initial position when robot returns (at backward_end)"""
        if self.current_pose is None or self.initial_pose is None:
            return

        pos_x = self.current_pose.position.x
        pos_y = self.current_pose.position.y

        # Calculate displacement in world frame
        dx_world = pos_x - self.initial_pose.position.x
        dy_world = pos_y - self.initial_pose.position.y

        # Get initial orientation
        initial_yaw = self.get_yaw_from_quaternion(self.initial_pose.orientation)

        # Transform displacement to robot's initial reference frame
        # longitudinal_drift = forward/backward drift (along initial heading)
        # lateral_drift = lateral drift (perpendicular to initial heading)
        longitudinal_drift = dx_world * math.cos(initial_yaw) + dy_world * math.sin(initial_yaw)
        lateral_drift = -dx_world * math.sin(initial_yaw) + dy_world * math.cos(initial_yaw)

        self.csv_writer.writerow(
            [iteration, cmd_vx, cmd_wz, pos_x, pos_y, longitudinal_drift, lateral_drift]
        )
        self.logfile.flush()
        
        self.get_logger().info(
            f"Iteration {iteration} complete - Drift from initial: longitudinal={longitudinal_drift:.4f}m, lateral={lateral_drift:.4f}m"
        )

    # ---------------------------
    # Control loop
    # ---------------------------
    def update(self):
        # Wait for first odometry message
        if self.current_pose is None:
            return

        # Record initial position
        if self.phase == "waiting_initial":
            self.initial_pose = self.current_pose
            self.get_logger().info(
                f"Initial position recorded: x={self.initial_pose.position.x:.3f}, y={self.initial_pose.position.y:.3f}"
            )
            self.phase = "forward"
            self.start_time = self.get_clock().now()
            self.current_iteration = 1
            return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        # State machine for forward/backward cycles
        if self.phase == "forward":
            if t < self.cmd_time_value:
                # Still in forward phase
                self.cmd_vel_msg.linear.x = self.cmd_vel_value
                self.cmd_vel_msg.angular.z = self.cmd_vel_ang_value
            else:
                # Forward phase complete, switch to backward
                self.phase = "backward"
                self.start_time = self.get_clock().now()

        elif self.phase == "backward":
            if t < self.cmd_time_value:
                # Still in backward phase
                self.cmd_vel_msg.linear.x = -self.cmd_vel_value
                self.cmd_vel_msg.angular.z = -self.cmd_vel_ang_value
            else:
                # Backward phase complete, log drift from initial position
                self.log_drift(
                    self.current_iteration, -self.cmd_vel_value, -self.cmd_vel_ang_value
                )
                self.current_iteration += 1

                # Check if we need more iterations
                if self.current_iteration <= self.num_iterations:
                    self.phase = "forward"
                    self.start_time = self.get_clock().now()
                else:
                    self.phase = "complete"
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_vel_msg)
                    self.get_logger().info("Drift test complete. Shutting down.")
                    self.cleanup_and_shutdown()
                    return

        # Publish command
        self.cmd_pub.publish(self.cmd_vel_msg)

    def cleanup_and_shutdown(self):
        if self.logfile:
            self.logfile.close()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = DriftTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.logfile:
            node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
