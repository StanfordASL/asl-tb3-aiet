#!/usr/bin/env python3
from enum import Enum, auto
from typing import Dict, Optional
from dataclasses import dataclass
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from std_msgs.msg import Bool

@dataclass
class Target:
    """Target information"""
    x: float
    y: float
    theta: float
    confidence: float

class TaskState(Enum):
    """States for the sequential navigation task"""
    SEARCHING = auto()      # Looking for targets
    NAV_TO_STOP = auto()    # Moving to stop sign
    STOP = auto()          # Waiting at stop sign
    NAV_TO_LIGHT = auto()   # Moving to traffic light
    FINISHED = auto()       # Task completed

class TaskExecutorBase(BaseController):
    """Base class for implementing see-think-act cycle"""
    
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        # State management
        self.current_state = TaskState.SEARCHING
        self.target_database: Dict[str, Target] = {}
        self.required_targets = {"stop sign", "traffic light"}
        self.start_wait_time: Optional[float] = None
        self.nav_success = False
        self.in_planning = False
        
        # Additional subscribers beyond BaseController
        # self.nav_success_sub = self.create_subscription(
        #     Bool, '/nav_success', self.nav_success_callback, 10)
            
        # Additional publisher beyond BaseController
        # self.cmd_nav_pub = self.create_publisher(
        #     TurtleBotState, '/cmd_nav', 10)
    
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
    
    def clear_database(self):
        """Clear the target database"""
        self.target_database.clear()

    def can_compute_control(self) -> bool:
        """Override from BaseController"""
        # Always allow control computation
        return True
    
    def perception_update(self):
        """Update perception - override if needed"""
        pass
    
    def process_perception(self, msg: TargetMarker):
        """Process new perception data"""
        raise NotImplementedError("Subclasses must implement process_perception")
        
    def decision_update(self):
        """Update decision making"""
        raise NotImplementedError("Subclasses must implement decision_update")
        
    def compute_action(self) -> TurtleBotControl:
        """Compute control command based on current state"""
        raise NotImplementedError("Subclasses must implement compute_action")