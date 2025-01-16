#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os
import numpy as np
from datetime import datetime
import json

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Create cv_bridge instance
        self.bridge = CvBridge()
        
        # Initialize data storage
        self.data_dir = self.declare_parameter('data_dir', os.path.expanduser('~/section_assets/driving_data')).value
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/image/decompressed',
            self.image_callback,
            5
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            5
        )
        self.command_sub = self.create_subscription(
            String,
            '/data_collection_command',
            self.command_callback,
            5
        )
        
        # Initialize state variables
        self.is_recording = False
        self.current_demo_dir = None
        self.frame_count = 0
        self.action_data = []
        
        # Initialize latest data
        self.latest_image = None
        self.latest_cmd_vel = None
        
        # Create timer for 10Hz operation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def create_demo_directory(self):
        # Create a new directory for this demonstration
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        demo_dir = os.path.join(self.data_dir, f'demo_{timestamp}')
        os.makedirs(os.path.join(demo_dir, 'images'), exist_ok=True)
        self.current_demo_dir = demo_dir
        self.frame_count = 0
        self.action_data = []
        return demo_dir

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image message: {str(e)}")

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def command_callback(self, msg):
        command = msg.data.lower()
        if command == "start":
            if not self.is_recording:
                self.create_demo_directory()
                self.is_recording = True
                self.get_logger().info("Started recording data")
            else:
                self.get_logger().warn("Already recording data")
        elif command == "stop":
            if self.is_recording:
                self.save_action_data()
                self.is_recording = False
                self.get_logger().info("Stopped recording data")
            else:
                self.get_logger().warn("Not currently recording data")

    def save_action_data(self):
        if not self.action_data:
            return
        
        # Save action data to JSON file
        action_file = os.path.join(self.current_demo_dir, 'actions.json')
        with open(action_file, 'w') as f:
            json.dump(self.action_data, f, indent=2)
        
        self.get_logger().info(f"Saved {len(self.action_data)} frames of data")

    def timer_callback(self):
        self.get_logger().info("Data collector initialized. Waiting for start command...")
        if self.is_recording and self.latest_image is not None and self.latest_cmd_vel is not None:
            # Save image
            image_filename = os.path.join(
                self.current_demo_dir, 
                'images', 
                f'frame_{self.frame_count:06d}.jpg'
            )
            cv2.imwrite(image_filename, self.latest_image)
            
            # Record action data
            action_data = {
                'frame_id': self.frame_count,
                'timestamp': self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
                'linear_velocity': {
                    'x': self.latest_cmd_vel.linear.x,
                    'y': self.latest_cmd_vel.linear.y,
                    'z': self.latest_cmd_vel.linear.z
                },
                'angular_velocity': {
                    'x': self.latest_cmd_vel.angular.x,
                    'y': self.latest_cmd_vel.angular.y,
                    'z': self.latest_cmd_vel.angular.z
                }
            }
            self.action_data.append(action_data)
            
            self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    collector = DataCollector()
    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()