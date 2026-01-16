#!/usr/bin/env python3
"""
Simple Navigation Module for TurtleBot3.

This module implements a finite state machine (FSM) based navigation system for a TurtleBot3 robot.
The robot performs a simple back-and-forth navigation task: moving forward 0.5 meters, then returning
to its origin position. The module integrates with ROS2 for communication and uses SLAM for localization
and mapping.

Key Features:
    - Finite state machine for sequential task execution
    - Integration with SLAM for localization and mapping
    - Navigation goal publishing and success monitoring
    - Autonomous navigation with trajectory planning

Author: ASL TurtleBot3 Team
"""

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
    """
    Simple navigation controller for TurtleBot3 using finite state machine.

    This class implements a sequential task executor that navigates the robot through
    a predefined sequence of waypoints. The robot starts at an origin position, moves
    forward 0.5 meters, then returns to the origin while rotating 180 degrees.

    The FSM has the following states:
        - ROBOT_START: Initial state, waiting for SLAM and map to be ready
        - ROBOT_SET_ORIGIN: Record the starting position
        - ROBOT_NAV_P1: Navigate to first waypoint (+0.5m in x-direction)
        - ROBOT_NAV_ORIGIN: Return to origin position
        - ROBOT_STANDBY: Final state after completing all navigation tasks

    Attributes:
        verbose (bool): Flag for enabling verbose logging output
        main_timer (Timer): Timer running at 10 Hz for the main control loop
        fsm_state (str): Current state of the finite state machine
        fsm_states (set): Set of valid FSM state names
        timestamp (float): Current timestamp in seconds
        phys_state (TurtleBotState): Current physical state (position and orientation) of the robot
        origin_state (TurtleBotState): Recorded origin position of the robot
        occupancy (OccupancyGrid): Current occupancy grid map from SLAM
        nav_success (bool): Flag indicating successful navigation to current goal
    """

    def __init__(self):
        """
        Initialize the SimpleNav node.

        Sets up publishers, subscribers, timers, and FSM state variables. Cancels the
        inherited control loop timer and creates a new main loop timer running at 10 Hz.
        """
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
        """
        Publish navigation goal commands to /cmd_nav using self.create_publisher()
        Syntax hint:
        self.example_pub = self.create_publisher(VariableType, "topic_to_publish_to", 10)
        See the Section 1 instructions for how to view the topics you can publish to
        You can call this publisher elswhere in your code after you have created it
        Hint: The variable type of the state of a TurtleBot is TurtleBotState.
        This variable contains x, y, and theta attributes in the global frame

        We are creating an object of type publisher. This object should become an attribute of this ROS node.
        Therefore, start by writing `self.cmd_nav_publisher = `.
        We tell python that we want to create an attribute called cmd_nav_publisher.
        Continue by typing self.create_pu... -> You should see a list of functions showing up.
        Select create_publisher with the arrow keys and press enter.
        You can also press Ctrl + spacebar to make these suggestions appear.
        Now type () and the tooltip for the function arguments appear.
        You can also right click on the function and go to definition to see the full function definition.
        Function arguments that are followed by = ... are optional -> You don't have to set them, but you might want to later.
        All other arguments are not optional and have to be set at function call.
        Start by typing ms... and you will see msg_type=. Press enter to autocomplete.
        Now you have to define the message type. 
        For you convenience, we already imported this message type for you. Scroll all the way up and check if you can find the correct import.
        Now, just complete msg_type=Tu...
        Continue by typing , and the tooltip should reappear.
        Next, type to... and autocomplete to topic=. What is our topic name?
        Make sure you put the topic name in "" (or '') and start with a leading /...
        Finish the call with , qos_profile=10.
        Congratulations, you wrote your first line of python today. You are amazing.
        """
        # --- YOUR CODE HERE ---
        # TODO: Create /cmd_nav publisher
    
        # --- END ---


    # Callbacks
    def phys_state_callback(self, msg):
        """
        Callback function for robot physical state updates.

        Updates the internal state representation whenever new state information is received
        from the /state topic.

        Args:
            msg (TurtleBotState): Message containing current x, y, and theta (orientation) of the robot
        """
        self.phys_state = msg

    def map_callback(self, msg):
        """
        Callback function for occupancy grid map updates.

        Updates the internal map representation whenever new map data is received from SLAM
        via the /map topic.

        Args:
            msg (OccupancyGrid): Occupancy grid map message containing environment information
        """
        self.occupancy = msg

    def nav_success_callback(self, msg: Bool):
        """
        Callback function for navigation success status.

        Updates the navigation success flag when the navigation stack reports completion
        (either successful or failed) of the current navigation goal.

        Args:
            msg (Bool): Boolean message indicating navigation success (True) or failure (False)
        """
        self.nav_success = msg.data
        self.get_logger().info(f"Navigation success: {self.nav_success}...")

    # Helper functions
    def pub_nav(self, x, y, theta):
        """
        Publish a navigation goal to the /cmd_nav topic.

        Creates and publishes a TurtleBotState message containing the target pose for the robot.
        Resets the nav_success flag to False and logs the navigation command. The navigation stack
        will receive this goal and plan a path to reach the target position and orientation.

        Args:
            x (float): Target x-coordinate in the global frame (meters)
            y (float): Target y-coordinate in the global frame (meters)
            theta (float): Target orientation angle in the global frame (radians)

        Note:
            Students must complete the TODO sections to set target_state attributes and publish
            the message to the /cmd_nav topic.
        """
        self.nav_success = False

        current_x = self.phys_state.x
        current_y = self.phys_state.y
        current_theta = self.phys_state.theta

        """
        Create and publish target state message to /cmd_nav.
        This message contains the desired pose (x, y, theta) for the TurtleBot. The passed goal message
        must be of type "TurtleBotState()". The navigation stack will plan and execute a path to reach this target
        """
        target_state = TurtleBotState()
        # --- YOUR CODE HERE ---
        # TODO: Update the attributes of target_state
        """
        You can set the attributes of objects with the dot operator `object.attribute = value`.
        We have to set the x, y, and theta attribute.
        """
        
        # --- END ---

        self.get_logger().info(f"Navigation target set:")
        self.get_logger().info(f"x: {current_x:.3f}, y: {current_y:.3f}, theta: {current_theta:.3f} --> x: {x:.3f}, y: {y:.3f}, theta: {theta:.3f}")

        """
        You can access class attributes in all class functions. 
        Notice that we created the class attribute cmd_nav_publisher in the constructor (__init__ function).
        Now, we can use it here.
        `self.` tells python that we want to access a class attribute.
        You can use the tooltip again to find the relevant function.
        """
        # --- YOUR CODE HERE ---
        # TODO: Publish target_state to your /cmd_nav publisher

        # --- END ---

    def get_time_sec(self):
        """
        Get current time in seconds.

        Returns:
            float: Current time in seconds since epoch, converted from nanoseconds
        """
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        return current_time_sec

    # FSM
    def switch_fsm_state(self, new_fsm_state):
        """
        Transition the finite state machine to a new state.

        Validates that the requested state exists in the set of valid FSM states before
        transitioning. Logs the state transition or a warning if the state is invalid.

        Args:
            new_fsm_state (str): Name of the state to transition to

        Side Effects:
            Updates self.fsm_state if the new state is valid
            Logs the state transition or warning message
        """
        if new_fsm_state in self.fsm_states:
            self.get_logger().info(f"{self.fsm_state} --> {new_fsm_state}")
            self.fsm_state = new_fsm_state
        else:
            self.get_logger().info(f"WARNING: Invalid fsm state requested: {new_fsm_state}")

    def modular_challenge_fsm(self):
        """
        Execute the finite state machine logic for the modular navigation challenge.

        Implements the state machine that controls the robot's navigation sequence:
        1. ROBOT_START: Wait for SLAM initialization and map to be available
        2. ROBOT_SET_ORIGIN: Record starting position and navigate forward
        3. ROBOT_NAV_P1: Wait for first navigation to complete, then navigate back
        4. ROBOT_NAV_ORIGIN: Wait for return navigation to complete
        5. ROBOT_STANDBY: Idle state after all tasks are completed

        The FSM transitions between states based on sensor readiness (phys_state, occupancy)
        and navigation success flags. Students must complete TODO sections to implement
        the actual navigation commands.

        Side Effects:
            Updates self.fsm_state through state transitions
            Updates self.origin_state when recording the starting position
        """
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
            # --- YOUR CODE HERE ---
            # TODO: Navigate 0.5m in the +x direction

            # --- END ---
            
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
                # --- YOUR CODE HERE ---
                # TODO: Navigate 0.5m in the -x direction

                # --- END ---
                
                self.switch_fsm_state("ROBOT_NAV_ORIGIN")
        elif self.fsm_state == "ROBOT_NAV_ORIGIN":
            if self.nav_success:
               self.switch_fsm_state("ROBOT_STANDBY") 
        elif self.fsm_state == "ROBOT_STANDBY":
            pass

    # Main loop
    def main_loop(self) -> None:
        """
        Main control loop executed at 10 Hz.

        This function is called periodically by the main_timer. It executes the finite
        state machine logic to progress through the navigation sequence.

        Side Effects:
            Calls modular_challenge_fsm() which may transition states and publish navigation goals
        """
        if self.verbose: self.get_logger().info(f"Entered main_loop...")
        self.modular_challenge_fsm()

    # Note: compute_control is vestigial for this script. You can ignore it.
    def compute_control(self) -> TurtleBotControl:
        """
        Compute control commands for the robot (vestigial method).

        This method is inherited from TaskExecutorBase but is not used in this implementation
        since navigation is handled by the navigation stack. It returns zero velocity commands.

        Returns:
            TurtleBotControl: Control message with zero linear and angular velocity

        Note:
            This method is vestigial for this script and can be ignored.
        """
        if self.verbose: self.get_logger().info(f"Entered compute_control...")
        control = TurtleBotControl()
        control.v = 0.0
        control.omega = 0.0
        return control

def main():
    """
    Main entry point for the simple navigation node.

    Initializes the ROS2 Python client library, creates a SimpleNav node instance,
    spins the node to process callbacks, and shuts down when complete.

    Side Effects:
        Initializes ROS2
        Creates and runs the SimpleNav node
        Blocks until node is shut down
        Shuts down ROS2 on exit
    """
    rclpy.init()
    node = SimpleNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
