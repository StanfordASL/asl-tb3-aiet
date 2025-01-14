#!/usr/bin/env python3
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_aiet.msg import TargetMarker
from dataclasses import dataclass

@dataclass
class Target:
    """Target information"""
    x: float
    y: float
    theta: float
    confidence: float


class TaskExecutorBase(BaseController):
    """Base class for implementing see-think-act cycle"""
    
    def __init__(self, node_name: str):
        super().__init__(node_name)

    def can_compute_control(self) -> bool:
        """Override from BaseController"""
        # Always allow control computation
        return True
    
    def perception_update(self):
        """Update perception - override if needed"""
        pass
    
    def process_perception(self):
        """Process new perception data"""
        raise NotImplementedError("Subclasses must implement process_perception")
        
    def decision_update(self):
        """Update decision making"""
        raise NotImplementedError("Subclasses must implement decision_update")
        
    def compute_action(self) -> TurtleBotControl:
        """Compute control command based on current state"""
        raise NotImplementedError("Subclasses must implement compute_action")