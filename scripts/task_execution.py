#!/usr/bin/env python3
from enum import Enum, auto
from task_base import TaskExecutorBase, Target
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from rclpy.parameter import Parameter
import rclpy
from typing import Dict, Optional
from std_msgs.msg import Bool

class TaskState(Enum):
    """States for the sequential navigation task"""
    SEARCHING = auto()      # Looking for targets
    NAV_TO_STOP = auto()    # Moving to stop sign
    STOP = auto()          # Waiting at stop sign
    NAV_TO_LIGHT = auto()   # Moving to traffic light
    FINISHED = auto()       # Task completed

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
        self.required_targets = {"stop sign", "traffic light"}
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
        """
        Initialize perception system and target tracking.
        TODO: Set up target subscriber and data storage.

        Hint:
        - Create subscriber to listen for target detections
        """
        
    
    # =========== Start of Helper Functions =========== #
    def nav_success_callback(self, msg: Bool):
        """Handle navigation completion"""
        if msg.data:
            self.control_timer.reset()
            
        self.nav_success = msg.data
    
    @property
    def database_complete(self) -> bool:
        """Check if all required targets are in database"""
        return all(target in self.target_database 
                  for target in self.required_targets)

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    
    @property
    def navigation_active(self) -> bool:
        return self.get_parameter("navigation_active").value

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
        if self.current_state == TaskState.SEARCHING and (next_state == TaskState.NAV_TO_LIGHT or next_state == TaskState.NAV_TO_STOP):
            self.current_state = next_state
            if next_state == TaskState.NAV_TO_LIGHT:
                self.start_navigation(self.target_database["traffic light"])
            elif next_state == TaskState.NAV_TO_STOP:
                self.start_navigation(self.target_database["stop sign"])
        elif self.current_state == TaskState.NAV_TO_LIGHT and next_state == TaskState.STOP:
            self.start_wait_time = self.get_current_time()
            self.current_state = next_state
            self.resume_control()
        elif self.current_state == TaskState.STOP and next_state == TaskState.NAV_TO_STOP:
            self.current_state = next_state
        elif self.current_state == TaskState.NAV_TO_STOP and next_state == TaskState.FINISHED:
            self.current_state = next_state
            self.get_logger().info("SUCCESS! Task completed!")
        else:
            self.get_logger().warn(f"Transition from {self.current_state} to {next_state} not supported. Skipping transition.")
    
    # =========== End of Helper Functions =========== #

    # =========== Start of student implementation below =========== #

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
        ########################
        # TODO: Student fill-in
        ########################
        pass

    def process_perception(self, target_msg: TargetMarker):
        """
        Process newly detected targets and add them to the database.
        
        This function is called whenever a new target is detected via target_callback.
        It should:
        1. Check if the detected target is new (not in database)
        2. If new, create a Target object with the target's:
        - x, y position
        - theta (orientation)
        - confidence
        3. Add the new Target object to target_database with target_type as key
        """
        self.current_target = target_msg
        ########################
        # TODO: Student fill-in
        ########################

        pass

    def compute_control(self) -> TurtleBotControl:
        """
        Main control loop implementing the sense-think-act paradigm.
        
        This function should:
        1. SENSE: Target detection is handled by target_callback
        2. THINK: Call decision_update() to process state transitions
        3. ACT: Return compute_action() to generate control commands
        
        Returns:
            TurtleBotControl: Control command with appropriate v and omega values
        """
        ########################
        # TODO: Student fill-in
        ########################
        # NOTE: See component is handled asynchronously whenever target is detected via target_callback

        pass

    def decision_update(self):
        """
        Update robot's state based on current conditions and transitions.
        
        State machine logic:
        1. SEARCHING -> NAV_TO_LIGHT:
        - Transition when database_complete is True
        
        2. NAV_TO_LIGHT:
        - If not navigating (not in_planning) and traffic light in database:
            * Start navigation to traffic light
        - Check for transition to STOP state
        
        3. STOP:
        - Track time spent waiting
        - Transition to NAV_TO_STOP after wait_duration
        
        4. NAV_TO_STOP:
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
        elif self.current_state == TaskState.NAV_TO_LIGHT and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.STOP and TODO:
            self.transition_state(TODO)
        elif self.current_state == TaskState.NAV_TO_STOP and TODO:
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
        3. Other states (NAV_TO_LIGHT, NAV_TO_STOP):
        - Navigation handled by navigation system
        
        Returns:
            TurtleBotControl: Message containing:
            - v: Linear velocity (m/s)
            - omega: Angular velocity (rad/s)
        """
        ########################
        # TODO: Student fill-in
        ########################

        pass

    # =========== End of student implementation below =========== #


def main():
    rclpy.init()
    node = SequentialTaskExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()