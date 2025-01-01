#!/usr/bin/env python3

import rclpy
import cv2
import torch
import torch.nn as nn
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from torchvision import transforms
from torchvision.models.detection import ssdlite320_mobilenet_v3_large
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from preprocess_data import extract_features, decode_velocities, predict
import pickle
import os

# TODO: Copy and paste your MLP class implementation from the training notebook here.
# Hint: Ensure your MLP class has the same structure and functionality as in the training notebook, 
#       including the __init__ and forward methods. This will allow the deployment script 
#       to use the trained model for inference.

class ILController(BaseController):
    def __init__(self):
        super().__init__("il_controller")
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Set up models
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load feature extractor
        self.feature_extractor = ssdlite320_mobilenet_v3_large(pretrained=True)
        self.feature_extractor.eval()
        self.feature_extractor.to(self.device)
        
        # Load IL model
        self.IL_model_path = os.path.expanduser("~/e2e_section/model_checkpoint.pth")
        self.il_model = self.load_il_model()
        self.il_model.eval()
        self.il_model.to(self.device)
        
        # Image preprocessing
        self.preprocess = transforms.Compose([transforms.ToTensor()])
        
        # Store latest image
        self.latest_image = None
        
        # Create image subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        # Load dataset statistics for normalization
        data_dir = os.path.expanduser("~/e2e_section/driving_data")
        with open(os.path.join(data_dir, 'processed_data.pkl'), 'rb') as f:
            dataset = pickle.load(f)
        self.feature_mean = np.mean(dataset['features'], axis=0)
        self.feature_std = np.std(dataset['features'], axis=0)

        # Store last computed control and setup counter for 1 Hz control
        self.last_control = TurtleBotControl()
        self.last_control.v = 0.0
        self.last_control.omega = 0.0
        self.control_counter = 0
    
    def load_il_model(self):
        checkpoint = torch.load(self.IL_model_path, map_location=self.device)
        
        # Get model class and create new instance
        ModelClass = checkpoint['model_class']
        model = ModelClass(**checkpoint['model_config'])
        
        # Load state
        model.load_state_dict(checkpoint['model_state_dict'])
        
        return model

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image message: {str(e)}')

    def compute_control(self) -> TurtleBotControl:
        # Only compute new control every 10 calls (1 Hz since base is 10 Hz)
        self.control_counter += 1
        if self.control_counter < 10:
            return self.last_control
        
        self.control_counter = 0

        if self.latest_image is None:
            self.get_logger().warn('No image received yet')
            return TurtleBotControl()
            
        try:
            # Preprocess image like in your original code
            img_cv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB)
            img_tensor = self.preprocess(img_cv)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)
            
            # Extract features using your original function
            feature_tensor = extract_features(self.feature_extractor, img_tensor)
            features_pooled = torch.amax(feature_tensor, dim=(2, 3)).squeeze(0)
            
            # Normalize features
            features_normalized = (features_pooled.cpu().numpy() - self.feature_mean) / self.feature_std
            features_tensor = torch.tensor(features_normalized, dtype=torch.float32).unsqueeze(0).to(self.device)
            
            # Get prediction from IL model
            action = predict(self.il_model, features_tensor, self.device)
            
            # Use your original decode_velocities function
            velocities = decode_velocities(action[np.newaxis, :])[0]
            self.get_logger().info(f"Velocities: {velocities}")
            
            # Create and return control message
            control = TurtleBotControl()
            control.v = float(velocities[0])
            control.omega = float(velocities[1])

            self.last_control = control
            return control
            
        except Exception as e:
            self.get_logger().error(f'Error computing control: {str(e)}')
            return TurtleBotControl()

def main(args=None):
    rclpy.init(args=args)
    controller = ILController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()