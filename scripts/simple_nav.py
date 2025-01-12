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

class SimpleNav(TaskExecutorBase):
    def __init__(self):
        super().__init__("sequential_task_executor")

        # For printing statements
        self.verbose = False

        # Stop inhereted control loop
        self.control_timer.cancel()

        # Create new main loop
        self.main_timer = self.create_timer(0.1, self.main_loop)

        # State machine variables
        self.fsm_state = "ROBOT_START"
        self.fsm_states = {"ROBOT_START",
                        "ROBOT_SET_ORIGIN",
                        "ROBOT_NAV_P1",
                        "ROBOT_NAV_ORIGIN",
                        "ROBOT_STANDBY"}

        # Timestamp variables
        self.timestamp = self.get_time_sec()

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

        # Publishers
        # TODO: Publish navigation goal commands to /cmd_nav using self.create_publisher()


    # Callbacks
    def phys_state_callback(self, msg):
        self.phys_state = msg

    def map_callback(self, msg):
        self.occupancy = msg

    def nav_success_callback(self, msg: Bool):
        self.nav_success = msg.data
        self.get_logger().info(f"Navigation success: {self.nav_success}...")

    # Helper functions
    def pub_nav(self, x, y, theta):
        self.nav_success = False

        current_x = self.phys_state.x
        current_y = self.phys_state.y
        current_theta = self.phys_state.theta

        """
        Create and publish target state message to /cmd_nav.
        This message contains the desired pose (x, y, theta) for the TurtleBot. The passed goal message
        must be of type "TurtleBotState()". The navigation stack will plan and execute a path to reach this target
        """
        # TODO: Add your code here

        self.get_logger().info(f"Navigation target set:")
        self.get_logger().info(f"x: {current_x}, y: {current_y}, theta: {current_theta} --> x: {x}, y: {y}, theta: {theta}")

    def get_time_sec(self):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        return current_time_sec

    # FSM
    def switch_fsm_state(self, new_fsm_state):
        if new_fsm_state in self.fsm_states:
            self.get_logger().info(f"{self.fsm_state} --> {new_fsm_state}")
            self.fsm_state = new_fsm_state
        else:
            self.get_logger().info(f"WARNING: Invalid fsm state requested: {new_fsm_state}")

    def modular_challenge_fsm(self):
        if self.fsm_state == "ROBOT_START":
            # Wait for robot to localize itself and build map
            if self.phys_state != None and self.occupancy != None:
                # Wait to ensure SLAM and map are ready
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5.0))
                self.switch_fsm_state("ROBOT_SET_ORIGIN")

        elif self.fsm_state == "ROBOT_SET_ORIGIN":
            # Remember origin state
            self.origin_state = self.phys_state
            
            """
            Send navigation command for the TurtleBot to move +0.5 meters in the x-direction.
            - x: moves +0.5 meters (i.e., self.phys_state.x + 0.5)
            - y: remains unchanged (i.e., self.phys_state.y)
            - theta: maintains current orientation
            Use your pub_nav() function to handle publishing this goal to the navigation stack
            """
            # TODO: Add your code here
            
            self.switch_fsm_state("ROBOT_NAV_P1")

        elif self.fsm_state == "ROBOT_NAV_P1":
            if self.nav_success:
                
                """
                Send navigation command for the TurtleBot to move -0.5 meters in the x-direction and rotate 180 degrees.
                - x: moves +0.5 meters (i.e., self.phys_state.x - 0.5)
                - y: remains unchanged (i.e., self.phys_state.y)
                - theta: rotates 180 degrees (i.e., self.phys_state.theta + π)
                Use your pub_nav() function to handle publishing this goal to the navigation stack
                """
                # TODO: Add your code here
                
                self.switch_fsm_state("ROBOT_NAV_ORIGIN")

        elif self.fsm_state == "ROBOT_NAV_ORIGIN":
            if self.nav_success:
               self.switch_fsm_state("ROBOT_STANDBY") 

        elif self.fsm_state == "ROBOT_STANDBY":
            pass

    # Main loop
    def main_loop(self) -> None:
        if self.verbose: self.get_logger().info(f"Entered main_loop...")
        self.modular_challenge_fsm()
    # --- END ---

    def compute_control(self) -> TurtleBotControl:
        """
        Override from BaseController - this is the main control loop.

        TODO: Implement the sense-think-act loop.
        Hint:
        - Use `self.perception_update()` to process perception data (sense).
        - Call `self.decision_update()` to update the robot's state (think).
        - Return the control command from `self.compute_action()` (act).
        """
        if self.verbose: self.get_logger().info(f"Entered compute_control...")
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