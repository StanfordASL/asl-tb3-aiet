#!/usr/bin/env python3
from enum import Enum, auto
from task_base import TaskExecutorBase, Target, TaskState
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from rclpy.parameter import Parameter
import rclpy
from typing import Dict, Optional
from std_msgs.msg import Bool


# From task_base.py:
# class TaskState(Enum):
#     """States for the sequential navigation task"""
#     SEARCHING = auto()          # Looking for targets
#     NAV_TO_TARGET_1 = auto()    # Moving to target 1
#     STOP = auto()               # Waiting at target 1
#     NAV_TO_TARGET_2 = auto()    # Moving to target 2
#     FINISHED = auto()           # Task completed

## From msg/TargetMarker.msg:
# # Target type (e.g., "stop sign" or "traffic light")
# string target_type
# 
# # Position in map frame
# float64 x
# float64 y
# float64 theta
# 
# # Confidence in detection/localization
# float64 confidence


class SequentialTaskExecutor(TaskExecutorBase):
    def __init__(self):
        super().__init__("sequential_task_executor")
        self.rotation_speed = 0.2  # rad/s for searching
        self.wait_duration = 5.0  # seconds
        
        # Add parameters like PerceptionController
        self.declare_parameter("active", True)
        self.declare_parameter("navigation_active", False)

        # State management
        self.current_state = TaskState.SEARCHING
        self.target_database: Dict[str, Target] = {}
        self.start_wait_time: Optional[float] = None
        self.current_target = None
        self.nav_success = False
        self.in_planning = False

        # Additional subscribers beyond BaseController
        self.nav_success_sub = self.create_subscription(
            Bool, '/nav_success', self.nav_success_callback, 10)
            
        # Additional publisher beyond BaseController
        self.cmd_nav_pub = self.create_publisher(
            TurtleBotState, '/cmd_nav', 10)
        
        # Create additional class properties
        self.target_sub = self.create_subscription(
            TargetMarker, '/target_marker', self.target_callback, 10)
        
    # =========== Start of Helper Functions =========== #
    def nav_success_callback(self, msg: Bool):
        """Handle navigation completion"""
        if msg.data:
            self.control_timer.reset()
            
        self.nav_success = msg.data
    
    @property
    def database_complete(self) -> bool:
        """Check if all required targets are in database"""
        return all(target in self.target_database for target in self.required_targets)

    @property
    def waited_long_enough(self) -> bool:
        """Check if the robot waited long enough at the stop sign."""
        if self.start_wait_time is None:
            self.get_logger().error("Requested access to property waited_long_enough before start_wait_time was set. Returning False.")
            return False
        return self.get_current_time() - self.start_wait_time >= self.wait_duration
    
    @property
    def navigation_successful(self) -> bool:
        """Check if navigation was successful."""
        return self.nav_success

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    
    @property
    def navigation_active(self) -> bool:
        return self.get_parameter("navigation_active").value

    @property
    def target_1_name(self) -> str:
        return self.get_parameter("target_classes").value[0]

    @property
    def target_2_name(self) -> str:
        return self.get_parameter("target_classes").value[1]

    def start_navigation(self, target: Target):
        """Start navigation mode"""
        self.send_nav_command(target)

        # Stop publishing controls
        self.control_timer.cancel()
        self.in_planning = True

    def resume_control(self):
        """Resume control after navigation"""
        self.in_planning = False
        self.nav_success = False

    def get_current_time(self):
        """Get current system time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9
    
    def target_callback(self, msg: TargetMarker):
        self.process_perception(msg)
        # self.get_logger().info(f"Found {msg.target_type}")
    
    def transition_state(self, next_state: TaskState):
        """Handle the transition from the self.current_state to next_state.
        
        Updates self.current_state and starts self.start_wait_time if needed.

        Args:
            next_state: the state to transition to.
        """
        self.get_logger().info(f"Transition from {self.current_state} to {next_state}...")
        if self.current_state == TaskState.SEARCHING and next_state == TaskState.NAV_TO_TARGET_1:
            self.current_state = next_state
            self.start_navigation(self.target_database[self.target_1_name])
        elif self.current_state == TaskState.NAV_TO_TARGET_1 and next_state == TaskState.STOP:
            self.start_wait_time = self.get_current_time()
            self.current_state = next_state
            self.resume_control()
        elif self.current_state == TaskState.STOP and next_state == TaskState.NAV_TO_TARGET_2:
            self.current_state = next_state
            self.start_navigation(self.target_database[self.target_2_name])
        elif self.current_state == TaskState.NAV_TO_TARGET_2 and next_state == TaskState.FINISHED:
            self.current_state = next_state
            self.get_logger().info("SUCCESS! Task completed!")
        else:
            self.get_logger().warn(f"Transition from {self.current_state} to {next_state} not supported. Skipping transition.")

    def process_perception(self, target_msg: TargetMarker):
        """
        Process newly detected targets and add them to the database.
        
        This function is called whenever a new target is detected via target_callback.
        It should:
        1. Check if the detected target is new (not in database)
            Tip: Investigate the database_complete() function for how we can check if an element is in the database. 
        2. If new, create a Target object with the target's:
        - x, y position
        - theta (orientation)
        - confidence
            Tip: the target class is defined in scripts/task_base.py.
        3. Add the new Target object to target_database with target_type as key
            Tip: target_database is a dictionary. 
            Learn more: https://www.geeksforgeeks.org/python/python-dictionary/
        """
        if target_msg.target_type not in self.target_database:
            new_target = Target(x=target_msg.x, y=target_msg.y, theta=target_msg.theta, confidence=target_msg.confidence)
            self.target_database[target_msg.target_type] = new_target
            self.get_logger().info(f"Added target {target_msg.target_type} with position {target_msg.x}, {target_msg.y} in the database.")

    def compute_control(self) -> TurtleBotControl:
        """
        Main control loop implementing the sense-think-act paradigm.
        
        This function should:
        1. SENSE: Target detection is handled by target_callback. Nothing to do here.
        2. THINK: Call decision_update() to process state transitions
        3. ACT: Return compute_action() to generate control commands
        
        Returns:
            TurtleBotControl: Control command with appropriate v and omega values
        """
        self.decision_update()
        return self.compute_action()
 
    def send_nav_command(self, target: Target):
        """
        Send navigation command to move robot to target location.
        
        Args:
            target (Target): Target object containing:
                - x, y: Target position coordinates
                - theta: Target orientation
        
        Steps:
        1. Create TurtleBotState message
        2. Set goal position (x, y) and orientation (theta)
        3. Publish command to navigation system via cmd_nav_pub
        """
        turtle_bot_state_msg = TurtleBotState()
        turtle_bot_state_msg.x = target.x
        turtle_bot_state_msg.y = target.y
        turtle_bot_state_msg.theta = target.theta
        self.cmd_nav_pub.publish(turtle_bot_state_msg)

    # =========== End of Helper Functions =========== #

    # =========== Start of student implementation below =========== #
    def decision_update(self):
        """
        Update robot's state based on current conditions and transitions.
        
        State machine logic:
        1. SEARCHING -> NAV_TO_TARGET_1:
        - Transition when database_complete is True
        
        2. NAV_TO_TARGET_1:
        - If not navigating (not in_planning) and traffic light in database:
            * Start navigation to traffic light
        - Check for transition to STOP state
        
        3. STOP:
        - Track time spent waiting
        - Transition to NAV_TO_TARGET_2 after wait_duration
        
        4. NAV_TO_TARGET_2:
        - If not navigating and stop sign in database:
            * Start navigation to stop sign
        - Check for transition to FINISHED state
        """
        ########################
        # TODO: Student fill-in
        ########################
        # Replace all TODO in the code and remove this line afterwards.
        TODO = False
        if self.current_state == TaskState.SEARCHING and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.NAV_TO_TARGET_1 and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.STOP and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.NAV_TO_TARGET_2 and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.FINISHED:
            # Print a fun little message if you want.
            self.get_logger().info(TODO)
        else:
            # We are still waiting for the last state to finish.
            pass
            
    def compute_action(self) -> TurtleBotControl:

        control = TurtleBotControl()

        """
        Generate control commands based on current state.
        
        State-specific behaviors:
        1. SEARCHING:
        - Rotate in place (v=0, omega=rotation_speed)
        2. STOP/FINISHED:
        - Remain stationary (v=0, omega=0)
        3. Other states (NAV_TO_TARGET_1, NAV_TO_TARGET_2):
        - Navigation handled by navigation system
        
        Returns:
            TurtleBotControl: Message containing:
            - v: Linear velocity (m/s)
            - omega: Angular velocity (rad/s)
        """
        ########################
        # TODO: Student fill-in
        ########################
        TODO = False
        if self.current_state == TaskState.SEARCHING:
            TODO
        elif self.current_state == TaskState.NAV_TO_TARGET_1 or self.current_state == TaskState.NAV_TO_TARGET_2:
            # This is handled by the navigation module.
            pass
        elif self.current_state == TaskState.STOP:
            TODO
        elif self.current_state == TaskState.FINISHED:
            TODO
        else:
            # Should not happen.
            self.get_logger().error(f"State {self.current_state} is not handled in compute_action()!")
        return control

    # =========== End of student implementation below =========== #


def main():
    rclpy.init()
    node = SequentialTaskExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()