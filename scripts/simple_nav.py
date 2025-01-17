#!/usr/bin/env python3
from task_base import TaskExecutorBase, TaskState, Target
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from rclpy.parameter import Parameter
import rclpy
from nav_msgs.msg import OccupancyGrid
import typing as T
import time
from std_msgs.msg import Bool
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SimpleNav(TaskExecutorBase):
    def __init__(self):
        super().__init__("sequential_task_executor")

        # For printing statements
        self.verbose = False

        # Stop inhereted control loop
        self.control_timer.cancel()

        # Create new main loop, running at 10 Hz
        self.main_timer = self.create_timer(0.1, self.main_loop)    

        # State machine variables
        self.fsm_state = "ROBOT_START"
        self.fsm_states = {"ROBOT_START",
                        "ROBOT_SET_ORIGIN",
                        "ROBOT_SEARCH_TARGET",
                        "ROBOT_NAV_TARGET",
                        "ROBOT_STANDBY"}

        # Timestamp variables
        self.timestamp = self.get_time_sec()
        self.nav_timeout = 10 # Seconds until navigator moves on

        # Target variables
        self.new_target_flag = False
        self.current_target_marker: T.Optional[TargetMarker] = None
        self.objective_target = "traffic light" # Can also be stop sign

        # Misc class variables
        self.phys_state: T.Optional[TurtleBotState] = None # Tracks robot state (x, y, theta)
        self.origin_state: T.Optional[TurtleBotState] = None # Records the starting position of the robot
        self.occupancy: T.Optional[OccupancyGrid] = None # Map data
        self.nav_success = False # True when robot navigates to SLAM target successfully
       
        # Subscriptions
        # Calls a function everytime the TurtleBot's position/orientation is calculated
        self.phys_state_sub = self.create_subscription(TurtleBotState, "/state", self.phys_state_callback, 10)
        # Calls a function everytime the TurtleBot's environment is updated
        self.occupancy_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        # Calls a function when the TurtleBot is done with navigating (success or failure)
        self.nav_state_sub = self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)
        # Calls function when TurtleBot sees target
        self.target_marker_sub = self.create_subscription(TargetMarker, "/target_marker", self.target_marker_callback, 10)

        # Publishers
        # Navigation publisher
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "cmd_nav", 10)
        # Print debugging publisher
        self.debug_print_pub = self.create_publisher(String, "/debug_print", 10)

        # Start message
        self.debug_print(f" ----- Starting state machine -----")
        self.debug_print(f"Objective target: {self.objective_target}")

    # Called when physical state is established
    def phys_state_callback(self, msg):
        self.phys_state = msg

    # Called when SLAM map is built
    def map_callback(self, msg):
        self.occupancy = msg

    # Called when target is detected by camera and LiDAR
    def target_marker_callback(self, msg):
        self.new_target_flag = True
        self.current_target_marker = msg

    # Called when navigation is complete
    def nav_success_callback(self, msg: Bool):
        self.nav_success = msg.data
        self.debug_print(f"Navigation success: {self.nav_success}...")

    # Helper functions
    def debug_print(self, msg_data: String) -> None:
        msg = String()
        msg.data = msg_data
        self.debug_print_pub.publish(msg)

    # Publishes a Twist velocity command
    def pub_cmd_vel(self, v: float, omega: float) -> None:
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    # Publishes a navigation waypoint
    def pub_nav(self, x, y, theta):
        self.nav_success = False

        current_x = self.phys_state.x
        current_y = self.phys_state.y
        current_theta = self.phys_state.theta

        target_state = TurtleBotState()
        target_state.x = x
        target_state.y = y
        target_state.theta = theta

        
        self.debug_print(f"Navigation target set:")
        self.debug_print(f"x: {current_x:.3f}, y: {current_y:.3f}, theta: {current_theta:.3f} --> x: {x:.3f}, y: {y:.3f}, theta: {theta:.3f}")
        # Publish target state to start navigating
        self.cmd_nav_pub.publish(target_state)

    # Gets the current time in seconds
    def get_time_sec(self):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        return current_time_sec

    # Swtiches between states and prints debug messages
    def switch_fsm_state(self, new_fsm_state):
        if new_fsm_state in self.fsm_states:
            self.debug_print(f"{self.fsm_state} --> {new_fsm_state}")
            self.fsm_state = new_fsm_state
        else:
            self.debug_print(f"WARNING: Invalid fsm state requested: {new_fsm_state}")

    # This is called at 10 Hz
    def modular_challenge_fsm(self):
        # Robot starts here
        if self.fsm_state == "ROBOT_START":
            # Wait for robot to localize itself and build map
            if self.phys_state != None and self.occupancy != None:
                # Wait to ensure SLAM and map are ready
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.0))
                self.switch_fsm_state("ROBOT_SET_ORIGIN")

        elif self.fsm_state == "ROBOT_SET_ORIGIN":
            # Remember origin state
            self.origin_state = self.phys_state

            # Start rotating to search for target
            self.switch_fsm_state("ROBOT_SEARCH_TARGET")

        elif self.fsm_state == "ROBOT_SEARCH_TARGET":
            self.pub_cmd_vel(0.0, -0.15)
            if self.new_target_flag:
                self.new_target_flag = False
                self.debug_print(f"Target found: {self.current_target_marker.target_type}")
                # If target matches objective target, navigate to it
                if self.current_target_marker.target_type == self.objective_target:
                    x = self.current_target_marker.x
                    y = self.current_target_marker.y
                    theta = self.current_target_marker.theta
                    self.pub_nav(x, y, theta)
                    self.timestamp = self.get_time_sec()
                    self.switch_fsm_state("ROBOT_NAV_TARGET")

        elif self.fsm_state == "ROBOT_NAV_TARGET":
            # If robots succeeds to navigate or times out, update objective target ad begin search
            if self.nav_success or (self.get_time_sec() - self.timestamp) > self.nav_timeout:
                if self.objective_target == "traffic light":
                    self.objective_target = "stop sign"
                    self.debug_print(f"Objective target: {self.objective_target}")
                    self.switch_fsm_state("ROBOT_SEARCH_TARGET")
                # End if current objective target is stop sign
                elif self.objective_target == "stop sign":
                    self.switch_fsm_state("ROBOT_STANDBY")

        # Stop
        elif self.fsm_state == "ROBOT_STANDBY":
            self.pub_cmd_vel(0.0, 0.0)

    # Main loop
    def main_loop(self) -> None:
        if self.verbose: self.debug_print(f"Entered main_loop...")
        self.modular_challenge_fsm()

    # Note: compute_control is vestigial for this script. You can ignore it.
    def compute_control(self) -> TurtleBotControl:
        if self.verbose: self.debug_print(f"Entered compute_control...")
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
