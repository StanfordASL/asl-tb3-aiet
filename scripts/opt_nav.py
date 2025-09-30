#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from asl_tb3_msgs.msg import TurtleBotState, ObjectDetection
import numpy as np

class SimpleNav(Node):
    def __init__(self):
        super().__init__("simple_nav")
        self.verbose = False  # Enable/disable debug prints

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        self.debug_print_pub = self.create_publisher(String, "/debug_print", 10)

        # Subscribers
        self.target_marker_sub = self.create_subscription(
            ObjectDetection, "/target_marker", self.target_marker_callback, 10
        )
        self.nav_success_sub = self.create_subscription(
            Bool, "/nav_success", self.nav_success_callback, 10
        )
        self.phys_state_sub = self.create_subscription(
            TurtleBotState, "/phys_state", self.phys_state_callback, 10
        )
        self.occupancy_sub = self.create_subscription(
            String, "/map", self.map_callback, 10
        )

        # Timers
        self.fsm_timer = self.create_timer(0.1, self.main_loop)  # 10 Hz

        # FSM State
        self.fsm_states = [
            "ROBOT_START",
            "ROBOT_SET_ORIGIN",
            "ROBOT_SEARCH_TRAFFIC_LIGHT",
            "ROBOT_NAV_TRAFFIC_LIGHT",
            "ROBOT_SEARCH_STOP_SIGN",
            "ROBOT_NAV_STOP_SIGN",
            "ROBOT_APPROACH_STOP_SIGN", # Ensure full entry for stop sign
            "ROBOT_HAPPY",
            "ROBOT_STANDBY",
        ]
        self.fsm_state = "ROBOT_START"

        # Target information
        self.objective_targets = ["traffic light", "stop sign"]
        self.objective_target_index = 0
        self.objective_target = self.objective_targets[self.objective_target_index]
        self.new_target_flag = False
        self.current_target_marker = None

        # Navigation flags
        self.nav_success = False
        self.nav_timeout = 15.0  # Increased timeout for robustness
        self.timestamp = 0.0
        self.stop_sign_entry_time = None

        # Robot state
        self.phys_state = None
        self.origin_state = None
        self.occupancy = None

        # Search parameters (Optimized for initial conditions)
        self.search_rotation_speed = 0.20
        self.search_sweep_angle = np.pi  # Sweep 180 degrees

        # Navigation parameters
        self.nav_acceptance_radius = 0.3
        self.dance_performed = False

        # Object detection threshold
        self.object_detection_threshold = 0.75

    # Callbacks
    def phys_state_callback(self, msg):
        self.phys_state = msg

    def map_callback(self, msg):
        self.occupancy = msg.data

    def target_marker_callback(self, msg):
        if msg.confidence > self.object_detection_threshold:
            self.new_target_flag = True
            self.current_target_marker = msg

    def nav_success_callback(self, msg: Bool):
        self.nav_success = msg.data
        self.debug_print(f"Navigation success: {self.nav_success}...")

    # Helper functions
    def debug_print(self, msg_data: String) -> None:
        msg = String()
        msg.data = msg_data
        self.debug_print_pub.publish(msg)

    def pub_cmd_vel(self, v: float, omega: float) -> None:
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    def pub_nav(self, x, y, theta):
        self.nav_success = False
        target_state = TurtleBotState()
        target_state.x = x
        target_state.y = y
        target_state.theta = theta
        self.debug_print(f"Navigating to: x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}")
        self.cmd_nav_pub.publish(target_state)

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def switch_fsm_state(self, new_fsm_state):
        if new_fsm_state in self.fsm_states:
            self.debug_print(f"{self.fsm_state} --> {new_fsm_state}")
            self.fsm_state = new_fsm_state
        else:
            self.debug_print(f"WARNING: Invalid fsm state requested: {new_fsm_state}")

    def is_in_target_area(self, target_x, target_y):
        if self.phys_state is None:
            return False
        distance = np.sqrt((self.phys_state.x - target_x)**2 + (self.phys_state.y - target_y)**2)
        return distance <= self.nav_acceptance_radius

    def modular_challenge_fsm(self):
        # Robot starts here
        if self.fsm_state == "ROBOT_START":
            if self.phys_state is not None and self.occupancy is not None:
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3.0))
                self.switch_fsm_state("ROBOT_SET_ORIGIN")

        elif self.fsm_state == "ROBOT_SET_ORIGIN":
            self.origin_state = self.phys_state
            self.switch_fsm_state("ROBOT_SEARCH_TRAFFIC_LIGHT")
            self.search_start_time = self.get_time_sec()
            self.current_search_angle = 0.0

        elif self.fsm_state == "ROBOT_SEARCH_TRAFFIC_LIGHT":
            # Prioritize upper-left/rear-left quadrant for traffic light
            if self.current_search_angle < self.search_sweep_angle:
                self.pub_cmd_vel(0.0, self.search_rotation_speed)
                self.current_search_angle += self.search_rotation_speed * 0.1
            else:
                self.pub_cmd_vel(0.0, 0.0) # Stop searching after sweep

            if self.new_target_flag and self.current_target_marker.target_type == "traffic light":
                self.new_target_flag = False
                self.debug_print("Traffic light found!")
                if abs(self.current_target_marker.x) < 5.0 and abs(self.current_target_marker.y) < 5.0:
                    self.pub_nav(self.current_target_marker.x, self.current_target_marker.y, self.current_target_marker.theta)
                    self.timestamp = self.get_time_sec()
                    self.switch_fsm_state("ROBOT_NAV_TRAFFIC_LIGHT")
                else:
                    self.debug_print("WARNING: Traffic light localization seems off.")

        elif self.fsm_state == "ROBOT_NAV_TRAFFIC_LIGHT":
            if self.nav_success:
                self.switch_fsm_state("ROBOT_SEARCH_STOP_SIGN")
            elif (self.get_time_sec() - self.timestamp) > self.nav_timeout:
                self.debug_print("Navigation to traffic light timed out.")
                self.switch_fsm_state("ROBOT_SEARCH_STOP_SIGN") # Try searching for the next target

        elif self.fsm_state == "ROBOT_SEARCH_STOP_SIGN":
            # Prioritize upper-right/rear-right quadrant for stop sign
            if self.current_search_angle < self.search_sweep_angle:
                self.pub_cmd_vel(0.0, -self.search_rotation_speed)
                self.current_search_angle += self.search_rotation_speed * 0.1
            else:
                self.pub_cmd_vel(0.0, 0.0)

            if self.new_target_flag and self.current_target_marker.target_type == "stop sign":
                self.new_target_flag = False
                self.debug_print("Stop sign found!")
                if abs(self.current_target_marker.x) < 5.0 and abs(self.current_target_marker.y) < 5.0:
                    self.pub_nav(self.current_target_marker.x, self.current_target_marker.y, self.current_target_marker.theta)
                    self.timestamp = self.get_time_sec()
                    self.switch_fsm_state("ROBOT_NAV_STOP_SIGN")
                else:
                    self.debug_print("WARNING: Stop sign localization seems off.")

        elif self.fsm_state == "ROBOT_NAV_STOP_SIGN":
            if self.nav_success:
                self.stop_sign_entry_time = self.get_time_sec()
                self.switch_fsm_state("ROBOT_APPROACH_STOP_SIGN")
            elif (self.get_time_sec() - self.timestamp) > self.nav_timeout:
                self.debug_print("Navigation to stop sign timed out.")
                self.switch_fsm_state("ROBOT_STANDBY")

        elif self.fsm_state == "ROBOT_APPROACH_STOP_SIGN":
            if self.current_target_marker is not None and self.is_in_target_area(self.current_target_marker.x, self.current_target_marker.y):
                self.pub_cmd_vel(0.0, 0.0) # Stop in the stop sign area
                self.debug_print("Entered stop sign area.")
                self.switch_fsm_state("ROBOT_HAPPY")
            elif (self.get_time_sec() - self.timestamp) > 5.0: # Small timeout for approach
                self.debug_print("Approach to stop sign timed out.")
                self.switch_fsm_state("ROBOT_STANDBY")

        elif self.fsm_state == "ROBOT_HAPPY":
            if self.get_time_sec() - self.stop_sign_entry_time >= 10.0 and not self.dance_performed:
                self.debug_print("Performing faster dance...")
                dance_duration = 0.5  # Seconds for each movement

                # Move 1: Short forward
                self.pub_cmd_vel(0.2, 0.0)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dance_duration))

                # Move 2: Short backward
                self.pub_cmd_vel(-0.2, 0.0)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dance_duration))

                # Move 3: Short spin
                self.pub_cmd_vel(0.0, 0.5)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dance_duration))

                # Stop
                self.pub_cmd_vel(0.0, 0.0)
                self.dance_performed = True
            elif self.dance_performed:
                self.switch_fsm_state("ROBOT_STANDBY")

        elif self.fsm_state == "ROBOT_STANDBY":
            self.pub_cmd_vel(0.0, 0.0)

    # Main loop
    def main_loop(self) -> None:
        if self.verbose: self.debug_print(f"FSM State: {self.fsm_state}")
        self.modular_challenge_fsm()

    def compute_control(self) -> TurtleBotControl:
        control = TurtleBotControl()
        control.v = 0.0
        control.omega = 0.0
        return control

def main():
    rclpy.init()
    node = SimpleNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
