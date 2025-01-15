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
        self.current_target = msg
        self.process_perception()
        # self.get_logger().info(f"Found {msg.target_type}")
    
    def transition_state(self, next_state):
        if self.current_state == TaskState.NAV_TO_LIGHT and self.nav_success:
            self.start_wait_time = self.get_current_time()
            self.current_state = next_state
            self.get_logger().info(f"Transition from NAV_TO_LIGHT to STOP")
            self.resume_control()
            
        elif self.current_state == TaskState.STOP:
            current_time = self.get_current_time()
            if self.start_wait_time is not None and current_time - self.start_wait_time >= self.wait_duration:
                self.current_state = next_state
                self.get_logger().info(f"Transition from STOP to NAV_TO_STOP")
                
        elif self.current_state == TaskState.NAV_TO_STOP and self.nav_success:
            self.current_state = next_state
            self.get_logger().info("Task complete!")
            self.resume_control()

        self.get_logger().info(f"DEBUG: My current state is {self.current_state}")
    
    # =========== End of Helper Functions =========== #

    def process_perception(self):
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
        ########################
        # TODO: Student fill-in
        ########################
        if self.current_target.target_type not in self.target_database:
            self.target_database[self.current_target.target_type] = Target(
                x=self.current_target.x,
                y=self.current_target.y,
                theta=self.current_target.theta,
                confidence=self.current_target.confidence
            )
        # self.get_logger().info(f"Added {self.current_target.target_type} to the database")

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

        # Think
        self.decision_update()

        # Act
        return self.compute_action()

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

        if self.current_state == TaskState.SEARCHING and self.database_complete:
            self.current_state = TaskState.NAV_TO_LIGHT
            self.get_logger().info("All Targets found!")
        
        elif self.current_state == TaskState.NAV_TO_LIGHT:
            if "traffic light" in self.target_database and not self.in_planning:
                self.get_logger().info("Navigating to traffic light")
                self.start_navigation(self.target_database["traffic light"])

            self.transition_state(TaskState.STOP)
        
        elif self.current_state == TaskState.STOP:
            current_time = self.get_current_time()
            if self.start_wait_time is not None and current_time - self.start_wait_time >= self.wait_duration:
                self.transition_state(TaskState.NAV_TO_STOP)

        elif self.current_state == TaskState.NAV_TO_STOP:
            if "stop sign" in self.target_database and not self.in_planning:
                self.start_navigation(self.target_database["stop sign"])
            
            self.transition_state(TaskState.FINISHED)
            
            
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
        if self.current_state == TaskState.SEARCHING:
            # Rotate to search for targets
            control.v = 0.0
            control.omega = self.rotation_speed
        
        elif self.current_state in [TaskState.STOP, TaskState.FINISHED]:
            # Stay stopped
            control.v = 0.0
            control.omega = 0.0

        return control
    
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
        goal = TurtleBotState()
        goal.x = target.x
        goal.y = target.y
        goal.theta = target.theta

        self.cmd_nav_pub.publish(goal)

def main():
    rclpy.init()
    node = SequentialTaskExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()