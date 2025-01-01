#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from std_msgs.msg import Bool


class PerceptionController(BaseController):
    def __init__(self, node_name: str = "perception_controller") -> None:
        super().__init__(node_name)
        self.declare_parameter("active", True)
        
        self.start_time = None
        self.omega = 0.2
        self.stop_time = 5.0

        self.image_sub = self.create_subscription(Bool, '/detector_bool', self.detect_callback, 10)
        self.image_time = None
        self.buffer_time = 3.0 # time for which we don't consider any detections

    @property
    def active(self) -> bool:
        return self.get_parameter("active").value

    def compute_control(self) -> TurtleBotControl:
        tb_control = TurtleBotControl()
        
        if self.active is False:
            tb_control.omega = 0.0
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
            else:
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
                if elapsed_time >= self.stop_time:
                    self.set_parameters([rclpy.Parameter("active", value=True)])
                    self.start_time = None
        else:
            tb_control.omega = self.omega

        return tb_control
    
    def detect_callback(self, msg: Bool):
        if msg.data and self.active:
            if self.image_time is None:
                self.image_time = self.get_clock().now().nanoseconds / 1e9
                self.set_parameters([rclpy.Parameter("active", value=False)])
            else:
                elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.image_time
                if elapsed_time >= self.stop_time + self.buffer_time:
                    self.image_time = None
        elif not msg.data:
            self.image_time = None


if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()
