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

    # =========== Start of student implementation below =========== #

    def send_nav_command(self, target: Target):
        """
        Send navigation command to move robot to target.

        TODO: Implement the navigation command publisher to move to targets.

        Implementation hints:
        1. Create a new TurtleBotState message
        2. Set the target pose:
           - x: target.x
           - y: target.y
           - theta: target.theta
        3. Publish using self.cmd_nav_pub
        4. Consider adding logging to track navigation commands
        """
        ########################
        # TODO: Student fill-in
        ########################
        pass

    def process_perception(self):
        """
        Process perception messages and update task state.

        TODO: Use perception input to determine the next task state.
        Hint:
        - Check the `self.current_state` to determine the current phase of the task.
        - Use `self.database_complete` to check if all targets have been found.
        - Update the state to `TaskState.NAV_TO_STOP` when ready to navigate to the stop sign.
        """
        ########################
        # TODO: Student fill-in
        ########################

        pass

    def compute_control(self) -> TurtleBotControl:
        """
        Override from BaseController - this is the main control loop.

        TODO: Implement the complete sense-think-act control loop for the robot.

        Implementation hints:
        1. Sensing: 
           - Target detection is handled asynchronously by target_callback
           - No additional sensing needed in this method
        2. Thinking:
           - Call self.decision_update() to process state transitions
           - This updates the robot's internal state based on current conditions
        3. Acting:
           - Return self.compute_action() to generate appropriate control commands
           - The control commands should match the current state
        4. Return Value:
           - Must return a TurtleBotControl message with appropriate v and omega values
        """
        ########################
        # TODO: Student fill-in
        ########################
        # NOTE: "See" component is handled asynchronously whenever target is detected via target_callback

        pass

    def decision_update(self):
        """
        Update decision making based on current state.

        TODO: Implement the state machine logic for transitioning between task states.

        Hints:
        1. State Transitions:
           SEARCHING -> NAV_TO_LIGHT:
           - Trigger when self.database_complete is True
           - Update current_state and begin navigation

           NAV_TO_LIGHT -> STOP:
           - Trigger when navigation is successful (self.nav_success)
           - Initialize self.start_wait_time with current time
           - Resume control authority

           STOP -> NAV_TO_STOP:
           - Wait for self.wait_duration seconds
           - Use self.get_current_time() to track elapsed time
           - Begin navigation to stop sign

           NAV_TO_STOP -> FINISHED:
           - Trigger when navigation is successful
           - Clean up and finalize task

        2. Navigation States:
           - Use self.in_planning to track active navigation
           - Call self.start_navigation() with appropriate target
           - Check self.nav_success to confirm arrival
           - Use self.resume_control() when navigation is complete

        3. Error Handling:
           - Verify targets exist in database before navigation
           - Handle cases where navigation fails
           - Implement appropriate logging for state transitions
        """
        ########################
        # TODO: Student fill-in
        ########################
        pass
            
            
    def compute_action(self) -> TurtleBotControl:
        """
        Compute control command based on current state.

        TODO: Implement state-specific control behaviors for the robot.

        Implementation hints:
        1. Create a new TurtleBotControl message for the command
        
        2. State-specific behaviors:
           SEARCHING:
           - Set omega = self.rotation_speed for rotation
           - Set v = 0.0 to rotate in place
           
           STOP/FINISHED:
           - Set both v and omega to 0.0 to remain stationary
           
           Other states:
           - Navigation is handled by the navigation system
           - Return zero velocities when not actively controlling

        3. Return Value:
           - Must return a TurtleBotControl message with appropriate
             v (linear velocity) and omega (angular velocity) values
        """
        pass

    # =========== End of student implementation below =========== #


def main():
    rclpy.init()
    node = SequentialTaskExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()