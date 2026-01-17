#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import csv
import os
import time
import math


class VelocityBatteryTestAvg(Node):
    def __init__(self):
        super().__init__("velocity_battery_test_avg")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("cmd_vel", 0.5)
        self.declare_parameter("cmd_vel_ang", 0.05)
        self.declare_parameter("cmd_time", 3.0)
        self.declare_parameter("num_runs", 10)

        self.cmd_vel_value = float(self.get_parameter("cmd_vel").value)
        self.cmd_time_value = float(self.get_parameter("cmd_time").value)
        self.cmd_vel_ang_value = float(self.get_parameter("cmd_vel_ang").value)
        self.num_runs = int(self.get_parameter("num_runs").value)

        self.get_logger().info(
            f"Starting test with cmd_vel={self.cmd_vel_value}, cmd_time={self.cmd_time_value}, num_runs={self.num_runs}"
        )

        # ---------------------------
        # ROS interfaces
        # ---------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(BatteryState, "/battery_state", self.battery_callback, 10)

        # Buffers
        self.cmd_vel_msg = Twist()
        self.battery_state = None
        self.initial_battery_value = None

        # For velocity calculation - track position at start/end of phase
        self.current_pose = None
        self.phase_start_pose = None
        self.phase_start_yaw = None

        # ---------------------------
        # Create output file
        # ---------------------------
        self.results_dir = os.path.expanduser("~/asl_tb3_aiet/results")
        os.makedirs(self.results_dir, exist_ok=True)

        # File will be created once battery value is known
        self.csv_writer = None
        self.logfile = None
        self.csv_writer_detailed = None
        self.logfile_detailed = None

        # Timing and state tracking
        self.start_time = self.get_clock().now()
        self.current_run = 0
        self.current_phase = "forward"  # "forward" or "backward"
        self.phase_start_time = 0.0  # Time when current phase started

        # Accumulator for averaging battery data only
        self.phase_voltage_sum = 0.0
        self.phase_percentage_sum = 0.0
        self.phase_sample_count = 0

        # Settling time after phase transition (seconds)
        self.settling_time = 0.5  # Skip first 0.5 seconds of each phase
        self.decel_time = 0.5  # Skip last 0.5 seconds of each phase (deceleration period)

        # Track if we've recorded the start position for current phase
        self.phase_start_recorded = False

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    # ---------------------------
    # Callbacks
    # ---------------------------
    def odom_callback(self, msg):
        # Just store the current pose
        self.current_pose = msg.pose.pose

    def battery_callback(self, msg):
        self.battery_state = msg
        if self.initial_battery_value is None:
            self.initial_battery_value = round(msg.percentage * 100, 1)  # percentage 0-1 → %
            self.setup_logfile()

    # ---------------------------
    # Setup logfile once battery is known
    # ---------------------------
    def setup_logfile(self):
        filename = f"vel_results_avg_bat_{self.initial_battery_value}_vel_{self.cmd_vel_value}_t_{self.cmd_time_value}_runs_{self.num_runs}.csv"
        path = os.path.join(self.results_dir, filename)

        self.logfile = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow(
            ["run", "phase", "cmd_vx", "avg_meas_vx", "avg_meas_wz", "avg_battery_voltage", "avg_battery_percentage"]
        )

        self.get_logger().info(f"Logging to: {path}")

    # ---------------------------
    # Control + Logging loop
    # ---------------------------
    def update(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9

        # Calculate which run and phase we're in
        cycle_time = 2 * self.cmd_time_value  # forward + backward
        total_time = self.num_runs * cycle_time

        if t >= total_time:
            # All runs complete, stop and shutdown
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)

            # If we have any remaining data from last phase, write it
            if self.phase_sample_count > 0:
                self.write_phase_average()

            self.get_logger().info("All experiments complete. Shutting down.")
            self.cleanup_and_shutdown()
            return

        # Determine current run and phase
        time_in_cycle = t % cycle_time
        new_run = int(t // cycle_time)

        if time_in_cycle < self.cmd_time_value:
            new_phase = "forward"
            self.cmd_vel_msg.linear.x = self.cmd_vel_value
            self.cmd_vel_msg.angular.z = self.cmd_vel_ang_value
        else:
            new_phase = "backward"
            self.cmd_vel_msg.linear.x = -self.cmd_vel_value
            self.cmd_vel_msg.angular.z = -self.cmd_vel_ang_value

        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_vel_msg)

        # Check if we've transitioned to a new phase
        if new_run != self.current_run or new_phase != self.current_phase:
            # Write average for previous phase if we have data
            if self.phase_sample_count > 0:
                self.write_phase_average()

            # Reset accumulators for new phase
            self.current_run = new_run
            self.current_phase = new_phase
            self.phase_start_time = t  # Record when this phase started
            self.phase_voltage_sum = 0.0
            self.phase_percentage_sum = 0.0
            self.phase_sample_count = 0
            self.phase_start_recorded = False
            self.phase_start_pose = None
            self.phase_start_yaw = None

        # Calculate time elapsed since phase started
        time_in_phase = t - self.phase_start_time

        # Only accumulate data during steady-state period
        # Skip settling time at the beginning and deceleration time at the end
        time_until_phase_end = self.cmd_time_value - time_in_phase
        in_steady_state = (time_in_phase >= self.settling_time and
                          time_until_phase_end >= self.decel_time)

        # Record the start position when we first enter steady state
        if in_steady_state and not self.phase_start_recorded and self.current_pose is not None:
            self.phase_start_pose = self.current_pose
            # Calculate yaw from quaternion
            siny_cosp = 2 * (self.current_pose.orientation.w * self.current_pose.orientation.z +
                            self.current_pose.orientation.x * self.current_pose.orientation.y)
            cosy_cosp = 1 - 2 * (self.current_pose.orientation.y * self.current_pose.orientation.y +
                                self.current_pose.orientation.z * self.current_pose.orientation.z)
            self.phase_start_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.phase_start_recorded = True
            self.steady_state_start_time = t

        # Accumulate battery data for averaging (only during steady state)
        if self.csv_writer is not None and self.battery_state is not None and in_steady_state:
            self.phase_voltage_sum += self.battery_state.voltage
            self.phase_percentage_sum += self.battery_state.percentage
            self.phase_sample_count += 1

    def write_phase_average(self):
        """Write the averaged data for the current phase to CSV"""
        if self.phase_sample_count > 0 and self.phase_start_pose is not None and self.current_pose is not None:
            # Calculate velocity from position change
            now = self.get_clock().now()
            t_now = (now - self.start_time).nanoseconds / 1e9
            dt = t_now - self.steady_state_start_time

            # Get end position and yaw
            dx = self.current_pose.position.x - self.phase_start_pose.position.x
            dy = self.current_pose.position.y - self.phase_start_pose.position.y

            # Calculate current yaw
            siny_cosp = 2 * (self.current_pose.orientation.w * self.current_pose.orientation.z +
                            self.current_pose.orientation.x * self.current_pose.orientation.y)
            cosy_cosp = 1 - 2 * (self.current_pose.orientation.y * self.current_pose.orientation.y +
                                self.current_pose.orientation.z * self.current_pose.orientation.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)

            # Transform velocity to robot frame (use average yaw)
            avg_yaw = (current_yaw + self.phase_start_yaw) / 2.0
            avg_vx = (dx * math.cos(avg_yaw) + dy * math.sin(avg_yaw)) / dt if dt > 0 else 0.0

            # Calculate angular velocity
            dyaw = current_yaw - self.phase_start_yaw
            # Normalize angle difference to [-pi, pi]
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            avg_wz = dyaw / dt if dt > 0 else 0.0

            # Average battery data
            avg_voltage = self.phase_voltage_sum / self.phase_sample_count
            avg_percentage = self.phase_percentage_sum / self.phase_sample_count

            cmd_vx = self.cmd_vel_value if self.current_phase == "forward" else -self.cmd_vel_value

            self.csv_writer.writerow(
                [
                    self.current_run + 1,  # 1-indexed for readability
                    self.current_phase,
                    cmd_vx,
                    avg_vx,
                    avg_wz,
                    avg_voltage,
                    avg_percentage,
                ]
            )
            self.logfile.flush()  # Ensure data is written immediately

    def cleanup_and_shutdown(self):
        if self.logfile:
            self.logfile.close()
        if self.logfile_detailed:
            self.logfile_detailed.close()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBatteryTestAvg()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.logfile:
            node.logfile.close()
        if node.logfile_detailed:
            node.logfile_detailed.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
