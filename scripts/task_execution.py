#!/usr/bin/env python3
from task_base import TaskExecutorBase, TaskState, Target
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_aiet.msg import TargetMarker
from rclpy.parameter import Parameter
import rclpy

class SequentialTaskExecutor(TaskExecutorBase):
    def __init__(self):
        super().__init__("sequential_task_executor")
        self.rotation_speed = 0.5  # rad/s for searching
        self.wait_duration = 5.0  # seconds
        
        # Add parameters like PerceptionController
        self.declare_parameter("active", True)
        self.declare_parameter("navigation_active", False)
        
        # Store timer period
        self.stored_timer_period = self.control_timer.timer_period_ns

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
    
    ########################
    # TODO: Student fill-in  
    ########################
    def process_perception(self, msg: TargetMarker):
        """
        Process perception messages and update task state.

        TODO: Use perception input to determine the next task state.
        Hint:
        - Check the `self.current_state` to determine the current phase of the task.
        - Use `self.database_complete` to check if all targets have been found.
        - Update the state to `TaskState.NAV_TO_STOP` when ready to navigate to the stop sign.
        """
        pass

    ########################
    # TODO: Student fill-in  
    ########################
    def compute_control(self) -> TurtleBotControl:
        """
        Override from BaseController - this is the main control loop.

        TODO: Implement the sense-think-act loop.
        Hint:
        - Use `self.perception_update()` to process perception data (sense).
        - Call `self.decision_update()` to update the robot's state (think).
        - Return the control command from `self.compute_action()` (act).
        """
        pass

    ########################
    # TODO: Student fill-in  
    ########################
    def decision_update(self):
        """
        Update decision making based on current state.

        TODO: Define how the system transitions between states.
        Hint:
        - For navigation states (e.g., `TaskState.NAV_TO_STOP`, `TaskState.NAV_TO_LIGHT`):
          - Check if the current navigation target is valid (e.g., the target exists in the database).
          - Initiate navigation to the target using an appropriate method (e.g., `self.start_navigation()`).
          - Ensure that navigation is complete by checking a relevant flag (e.g., `self.nav_success`) before transitioning to the next state.
          - Once navigation is complete, return control authority to the controller using a relevant method (e.g., `self.resume_control()`).

        - For waiting states (e.g., `TaskState.STOP`):
          - Track the time spent in the current state.
          - Transition to the next state when the wait duration (e.g., `self.wait_duration`) is reached.

        - For final states (e.g., `TaskState.FINISHED`):
          - Ensure all tasks are complete and clean up resources (e.g., clear the target database).

        Ensure that transitions are logical and depend on the current state, task conditions, and system flags.
        """
        pass
    
    ########################
    # TODO: Student fill-in  
    ########################
    def compute_action(self) -> TurtleBotControl:
        """Compute control command based on current state"""
        control = TurtleBotControl()
        
        """
        Compute control command based on current state.

        TODO: Define movement commands for the robot based on its state.
        Hint:
        - For `TaskState.SEARCHING`, set the angular velocity (`control.omega`) or the linear velocity (self.v) to find target.
        - For `TaskState.STOP` or `TaskState.FINISHED`, set both `control.v` and `control.omega` appropriately
        - Use the `TurtleBotControl` message to assign velocity commands.
        """
                    
        return control

def main():
    rclpy.init()
    node = SequentialTaskExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()