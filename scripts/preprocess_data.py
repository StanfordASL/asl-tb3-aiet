import cv2
import torch
from torchvision import transforms
from torchvision.models.detection import ssdlite320_mobilenet_v3_large
import os
import json
import numpy as np
from pathlib import Path
import pickle
from os.path import join

COCO_LABELS = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

def encode_velocity(velocities):
    """
    Convert [linear_vel, angular_vel] pairs to [is_linear, is_angular, value] format
    """
    encoded = []
    linear_vel, angular_vel = velocities
    if linear_vel != 0:
        encoded.append([1, 0, linear_vel])
    else:
        encoded.append([0, 1, angular_vel])

    return np.array(encoded).flatten()

# To use the outputs during inference:
def decode_velocities(encoded):
    """
    Convert [is_linear, is_angular, value] back to [linear_vel, angular_vel]
    """
    velocities = []
    for is_linear, is_angular, value in encoded:
        if is_linear > is_angular:  # You might want to use argmax here depending on your model output
            velocities.append([value, 0.0])
        else:
            velocities.append([0.0, value])
    return np.array(velocities)

def extract_features(model, img_tensor):
    """
    Extract features from the second-to-last layer of the ssdlite320_mobilenet_v3_large model.
    """
    with torch.no_grad():
        features = model.backbone(img_tensor)["0"]
    
    return features

def process_demo_folder(model, demo_path, device, preprocess):
    """
    Process a single demonstration folder
    """
    # Load actions
    try:
        with open(os.path.join(demo_path, 'actions.json'), 'r') as f:
            actions = json.load(f)
    except FileNotFoundError:
        print(f"No actions.json found in {demo_path}")
        return None, None
    
    features_list = []
    targets_list = []
    
    # Process each image
    images_path = os.path.join(demo_path, 'images')
    for action in actions:
        # Get corresponding image
        img_filename = f"frame_{action['frame_id']:06d}.jpg"
        img_path = os.path.join(images_path, img_filename)
        
        if not os.path.exists(img_path):
            print(f"Warning: Image {img_filename} not found")
            continue
            
        # Load and preprocess image
        img_cv = cv2.imread(img_path)
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)
        img_tensor = preprocess(img_cv)
        img_tensor = img_tensor.unsqueeze(0).to(device)
        
        # Extract features
        feature_tensor = extract_features(model, img_tensor)
        features_pooled = torch.amax(feature_tensor, dim=(2, 3)).squeeze(0)
        
        # Extract target (actions)
        target = np.array([
            action['linear_velocity']['x'],
            action['angular_velocity']['z']
        ])
        
        features_list.append(features_pooled.cpu().numpy())
        targets_list.append(encode_velocity(target))
    
    return np.array(features_list), np.array(targets_list)

def predict(model, features, device):
    """
    Perform inference on the given features using the model.
    """
    features = features.to(device)  # Move features to device
    with torch.no_grad():  # No gradient computation for inference
        classification_logits, regression_output = model(features)

    # Get classification predictions (type of velocity)
    classification_probs = torch.softmax(classification_logits, dim=1).cpu().numpy()

    # Get regression prediction (velocity value)
    regression_pred = regression_output.item()

    return np.concatenate([classification_probs[0], [regression_pred]])

def process_dataset(data_dir):
    # Set up model
    model = ssdlite320_mobilenet_v3_large(pretrained=True)
    model.eval()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = model.to(device)
    
    # Set up preprocessing
    preprocess = transforms.Compose([transforms.ToTensor()])
    
    # Initialize lists for all data
    all_features = []
    all_targets = []
    
    # Process all demonstration folders
    for demo_folder in os.listdir(data_dir):
        demo_path = os.path.join(data_dir, demo_folder)
        if not os.path.isdir(demo_path):
            continue
            
        print(f"Processing {demo_folder}...")
        features, targets = process_demo_folder(model, demo_path, device, preprocess)
        
        if features is not None and targets is not None:
            all_features.append(features)
            all_targets.append(targets)
    
    # Combine all data
    if all_features:
        all_features = np.concatenate(all_features, axis=0)
        all_targets = np.concatenate(all_targets, axis=0)
        
        # Create a dictionary of the data
        dataset = {
            'features': np.array(all_features),
            'targets': np.array(all_targets)
        }

        # Save as pickle file in the driving_data directory
        save_path = os.path.join(data_dir, 'processed_data.pkl')
        with open(save_path, 'wb') as f:
            pickle.dump(dataset, f, protocol=3)
        
        print(f"Dataset saved to {save_path}")
        print(f"Dataset contains {len(all_features)} samples")
        print(f"Feature shape: {all_features.shape}")
        print(f"Target shape: {all_targets.shape}")
    else:
        print("No data was processed!")

def load_processed_data(data_dir):
    with open(join(data_dir, 'processed_data.pkl'), 'rb') as f:
        dataset = pickle.load(f)
    
    features = dataset['features']
    targets = decode_velocities(dataset['targets'])

    print(f"Dataset saved to {data_dir}")
    print(f"Dataset contains {len(features)} samples")
    print(f"Feature shape: {np.array(features).shape}")
    print(f"Target shape: {np.array(targets).shape}")

if __name__ == "__main__":
    data_dir = os.path.expanduser("~/e2e_section/driving_data")
    process_dataset(data_dir)

    # load_processed_data(data_dir)