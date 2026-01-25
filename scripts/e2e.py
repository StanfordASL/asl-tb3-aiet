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
from preprocess_data import ILPreProcessManager
import pickle
import os
import torchvision
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

######## Copy and paste the definition of your behavior cloning policy here ########

# TODO Replace the following combined_loss function and MLP class definition if you changed the model architecture in your Google colab notebook!

# Combined Loss Function
def combined_loss(classification_predict, classification_targets, regression_predict, regression_targets):
    """
    # This function needs to:
    # 1. Calculate classification loss (for predicting velocity type)
    #
    # 2. Calculate regression loss (for predicting velocity)
    #
    # 3. Combine the losses (think: how should they be weighted?)
    #
    # Args:
    #    classification_predict: Raw model outputs for velocity class prediction
    #    classification_targets: True velocity class labels
    #    regression_predict: Predicted velocity values
    #    regression_targets: True velocity values
    #
    # Hints:
    # - Check input dimensions match loss function expectations
    # - Consider if both losses should be weighted equally
    # - Define regression and loss functions using the functions in https://pytorch.org/docs/stable/nn.html#loss-functions
    """
    # 1. Calculate classification loss (using Cross-Entropy Loss)
    classification_loss_fn = nn.CrossEntropyLoss()
    classification_loss = classification_loss_fn(classification_predict, classification_targets)

    # 2. Calculate regression loss (using Mean Squared Error)
    regression_loss_fn = nn.MSELoss()
    # Ensure regression_predict and regression_targets have the same shape
    regression_targets = regression_targets.view_as(regression_predict)
    regression_loss = regression_loss_fn(regression_predict, regression_targets)

    # 3. Combine the losses (simple sum - you can adjust weights if needed)
    combined_loss = classification_loss + regression_loss

    return combined_loss


class MLP(nn.Module):
    def __init__(self, input_size, hidden_size):
        super(MLP, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.dropout_ratio = 0.5  # Example dropout ratio

        """
        Initialize the MLP model.

        Args:
        - input_size (int): Size of the input feature vector.
        - hidden_size (int): Number of neurons in the first hidden layer.
        - Note: You can add more hidden layers as required by adding more 'hidden_size' arguments in __init__.

        - Define fully connected layers (`nn.Linear`).
        - Define non-linearity for activation for activation (see: https://pytorch.org/docs/stable/nn.html#non-linear-activations-weighted-sum-nonlinearity).

        Hint:
        - For layer dimensions, follow these:
          * Input to hidden layer 1: `input_size -> hidden_size_1`
          * Hidden layer 1 to hidden layer 2: `hidden_size_1 -> hidden_size_2`
          * Final outputs:
            - Classification layer: `hidden_size_2 -> 2` (2 classes)
            - Regression layer: `hidden_size // 2 -> 1` (velocity value)
          * Use batch normalization (i.e., nn.BatchNorm1d(self.hidden_size)) and weight dropout (i.e., nn.Dropout(self.dropout_ratio)) to stabilize training.
        """
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.bn1 = nn.BatchNorm1d(hidden_size)
        self.dropout1 = nn.Dropout(self.dropout_ratio)

        # Example: Adding a second hidden layer
        self.fc2 = nn.Linear(hidden_size, hidden_size // 2)
        self.bn2 = nn.BatchNorm1d(hidden_size // 2)
        self.dropout2 = nn.Dropout(self.dropout_ratio)


        # Output layers
        self.classification_output = nn.Linear(hidden_size // 2, 2)  # 2 classes: linear, angular
        self.regression_output = nn.Linear(hidden_size // 2, 1)     # 1 output: velocity value

    def forward(self, x):
        """
        Forward pass of the MLP.

        Args:
        - x (Tensor): Input tensor of shape (batch_size, input_size).

        Returns:
        - classification_logits (Tensor): Logits for classification task, i.e., linear or angular velocity (batch_size, 2).
        - regression_output (Tensor): Scalar values for regression task, i.e., velocity value (batch_size, 1).

        - Pass the input `x` through the layers defined in `__init__`.
        - Apply ReLU activation and dropout after each layer (except the output layers).
        - Separate the output into two branches:
          * One for classification logits
          * One for regression output

        Hint:
        - Use the layers and activations in the following order:
          * Input -> Layer 1 -> ReLU
          * Layer 1 -> Layer 2 -> ReLU
          * Layer 2 -> Separate into classification and regression branches
          * If using batch norm, place it between layer and non-linear activation
          * If using dropout, place it after activation.
        """
        x = self.fc1(x)
        x = self.bn1(x)
        x = torch.relu(x)
        x = self.dropout1(x)

        x = self.fc2(x)
        x = self.bn2(x)
        x = torch.relu(x)
        x = self.dropout2(x)

        classification_logits = self.classification_output(x)
        regression_output = self.regression_output(x)

        return classification_logits, regression_output

######################## End of behavior cloning definition #########################


class ILController(BaseController):
    def __init__(self):
        super().__init__("il_controller")
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Set up models
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load feature extractor
        model_path = os.path.expanduser("~/section_assets/finetuned_ssd_model.pkl")
        if os.path.exists(model_path):
            self.get_logger().info("Using finetuned model to preprocess features for inference")
            # Use COCO_V1 weights like in training
            self.feature_extractor = ssdlite320_mobilenet_v3_large(
                weights=torchvision.models.detection.SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
            )
            state_dict = torch.load(model_path, map_location=self.device, weights_only=True)
            self.feature_extractor.load_state_dict(state_dict)
        else:
            self.get_logger().info("Using default SSD model to preprocess features for inference")
            self.feature_extractor = ssdlite320_mobilenet_v3_large(pretrained=True)

        self.feature_extractor.eval()
        self.feature_extractor.to(self.device)
        
        # Load IL model
        self.IL_model_path = os.path.expanduser("~/section_assets/IL_model_checkpoint.pth")
        self.il_model = torch.load(self.IL_model_path, map_location=self.device)['model']
        self.il_model.eval()
        self.il_model.to(self.device)
        
        # Image preprocessing
        self.preprocess = transforms.Compose([transforms.ToTensor()])
        
        # Store latest image
        self.latest_image = None

        # Define a Best Effort QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create image subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            self.qos_profile
        )

        # Load dataset statistics for normalization
        data_dir = os.path.expanduser("~/section_assets/driving_data")
        with open(os.path.join(data_dir, 'processed_data.pkl'), 'rb') as f:
            dataset = pickle.load(f)
        self.feature_mean = np.mean(dataset['features'], axis=0)
        self.feature_std = np.std(dataset['features'], axis=0)

        # Store last computed control and setup counter for 1 Hz control
        self.last_control = TurtleBotControl()
        self.last_control.v = 0.0
        self.last_control.omega = 0.0
        self.control_counter = 0

        # Statistics tracking
        self.execution_start_time = self.get_current_time()

    def get_current_time(self):
        """Get current system time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9

    def print_stats(self):
        """Print execution statistics including total execution time."""
        current_time = self.get_current_time()
        total_execution_time = current_time - self.execution_start_time

        self.get_logger().info("=" * 60)
        self.get_logger().info("EXECUTION STATISTICS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Total Execution Time: {total_execution_time:.2f} seconds")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image message: {str(e)}')

    def compute_control(self) -> TurtleBotControl:
        # Only compute new control every 10 calls (1 Hz since base is 10 Hz)
        # self.control_counter += 1
        # if self.control_counter < 10:
        #     return self.last_control
        # 
        # self.control_counter = 0

        if self.latest_image is None:
            self.get_logger().warn('No image received yet')
            return TurtleBotControl()
            
        try:
            # Preprocess image like in your original code
            img_cv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB)
            img_tensor = self.preprocess(img_cv)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)
            
            # Extract features using your original function
            ILEpreprocessor = ILPreProcessManager()
            feature_tensor = ILEpreprocessor.extract_features(self.feature_extractor, img_tensor)
            features_pooled = torch.amax(feature_tensor, dim=(2, 3)).squeeze(0)
            
            # Normalize features
            features_normalized = (features_pooled.cpu().numpy() - self.feature_mean) / self.feature_std
            features_tensor = torch.tensor(features_normalized, dtype=torch.float32).unsqueeze(0).to(self.device)
            
            # Get prediction from IL model
            action = ILEpreprocessor.predict(self.il_model, features_tensor, self.device)
            
            # Use your original decode_velocities function
            velocities = ILEpreprocessor.decode_velocities(action[np.newaxis, :])[0]
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
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Execution interrupted by user (Ctrl+C)")
    finally:
        # Print statistics before shutdown
        controller.print_stats()
        rclpy.shutdown()

if __name__ == '__main__':
    main()