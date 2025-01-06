import cv2
import torch
from torchvision import transforms
from torchvision.models.detection import ssdlite320_mobilenet_v3_large
from PIL import Image, ImageOps, ImageDraw
import os
import json
import numpy as np
from pathlib import Path
import pickle
from os.path import join
import copy
import torchvision

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

class PreProcessManager:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
    
    def _init_feature_extractor(self):
        """Initialize and return model for feature extraction."""
        pass

    def _process_image(self, img_path):
        """Load and preprocess image from path into tensor format."""
        pass

    def _print_dataset_info(self, path, dataset):
        """Print dataset statistics and metadata."""
        pass

    def extract_features(self, feature_extractor, img_tensor):
        """Extract features from preprocessed image tensor using feature extractor."""
        pass


class ILPreProcessManager(PreProcessManager):
    def __init__(self):
        super(ILPreProcessManager, self).__init__()
        self.base_dir = Path.home() / "section_assets"
        self.cv_checkpoint_name = "finetuned_ssd_model"
        self.cv_model_path = self.base_dir / f"{self.cv_checkpoint_name}.pkl"

        # Load feature extractor
        self.feature_extractor = self._init_feature_extractor()
        self.preprocess = transforms.Compose([transforms.ToTensor()])

    # Loading default features or finetuned features
    def _init_feature_extractor(self):
        if os.path.exists(self.cv_model_path):
            print("Using finetuned model to preprocess features")
            # Use COCO_V1 weights like in training
            model = ssdlite320_mobilenet_v3_large(
                weights=torchvision.models.detection.SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
            )
            state_dict = torch.load(self.cv_model_path, map_location=self.device, weights_only=True)
            model.load_state_dict(state_dict)
        else:
            print("Using default SSD model to preprocess features")
            model = ssdlite320_mobilenet_v3_large(pretrained=True)
        
        model.eval()
        return model.to(self.device)

    # Velocity encoding/decoding methods
    def encode_velocity(self, velocities):
        encoded = []
        linear_vel, angular_vel = velocities
        encoded.append([1, 0, linear_vel] if linear_vel != 0 else [0, 1, angular_vel])
        return np.array(encoded).flatten()

    def decode_velocities(self, encoded):
        return np.array([[value if is_linear > is_angular else 0.0, 
                         0.0 if is_linear > is_angular else value] 
                        for is_linear, is_angular, value in encoded])

    # Feature extraction
    def extract_features(self, feature_extractor, img_tensor):
        with torch.no_grad():
            return feature_extractor.backbone(img_tensor)["0"]

    def _process_image(self, img_path):
        img_cv = cv2.imread(img_path)
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)
        img_tensor = self.preprocess(img_cv)
        return img_tensor.unsqueeze(0).to(self.device)

    # Data processing methods
    def process_demo_folder(self, demo_path):
        try:
            with open(os.path.join(demo_path, 'actions.json'), 'r') as f:
                actions = json.load(f)
        except FileNotFoundError:
            print(f"No actions.json found in {demo_path}")
            return None, None

        features_list, targets_list = [], []
        images_path = os.path.join(demo_path, 'images')

        for action in actions:
            img_path = os.path.join(images_path, f"frame_{action['frame_id']:06d}.jpg")
            if not os.path.exists(img_path):
                continue

            img_tensor = self._process_image(img_path)
            feature_tensor = self.extract_features(self.feature_extractor, img_tensor)
            features_pooled = torch.amax(feature_tensor, dim=(2, 3)).squeeze(0)

            target = np.array([action['linear_velocity']['x'], 
                             action['angular_velocity']['z']])

            features_list.append(features_pooled.cpu().numpy())
            targets_list.append(self.encode_velocity(target))

        return map(np.array, (features_list, targets_list))

    def predict(self, model, features, device):
        with torch.no_grad():
            classification_logits, regression_output = model(features.to(device))
            classification_probs = torch.softmax(classification_logits, dim=1).cpu().numpy()
            return np.concatenate([classification_probs[0], [regression_output.item()]])

    def process_dataset(self, data_dir):
        all_features, all_targets = [], []
        
        for demo_folder in os.listdir(data_dir):
            demo_path = os.path.join(data_dir, demo_folder)
            if not os.path.isdir(demo_path):
                continue

            features, targets = self.process_demo_folder(demo_path)
            print(f"Processing {demo_folder}...")
            if features is not None:
                all_features.append(features)
                all_targets.append(targets)

        if not all_features:
            print("No data was processed!")
            return

        dataset = {
            'features': np.concatenate(all_features, axis=0),
            'targets': np.concatenate(all_targets, axis=0)
        }

        save_path = os.path.join(data_dir, 'processed_data.pkl')
        with open(save_path, 'wb') as f:
            pickle.dump(dataset, f, protocol=3)

        self._print_dataset_info(save_path, dataset)

    def load_processed_data(self, data_dir):
        with open(os.path.join(data_dir, 'processed_data.pkl'), 'rb') as f:
            dataset = pickle.load(f)
        dataset['targets'] = self.decode_velocities(dataset['targets'])
        self._print_dataset_info(data_dir, dataset)
        return dataset

    def _print_dataset_info(self, path, dataset):
        print(f"Dataset saved to {path}")
        print(f"Dataset contains {len(dataset['features'])} samples")
        print(f"Feature shape: {dataset['features'].shape}")
        print(f"Target shape: {dataset['targets'].shape}")

class CVPreProcessManager(PreProcessManager):
   def __init__(self):
       super().__init__()
       self.preprocess = transforms.Compose([transforms.ToTensor()])
       self.img_size = (320, 320) # TODO: Update image size here
       # Define correct ids based on COCO convention
       self.coco_id_to_name_dict = {10: "traffic light", 13: "stop sign"}
       self.coco_name_to_id_dict = {"traffic light": 10, "stop sign": 13}


   def _process_image(self, img_path):
       img = Image.open(img_path).convert("RGB")

       # Rotate image
       vert_img = ImageOps.exif_transpose(img)
       
       # Resize image
       resize_transform = transforms.Resize(self.img_size)
       resized_img = resize_transform(vert_img)
       return resized_img, vert_img.size
   
   def resize_bbox(self, orig_img_size, new_img_size, orig_bbox):
        orig_img_height = orig_img_size[0]
        orig_img_width = orig_img_size[1]
        new_img_height = new_img_size[0]
        new_img_width = new_img_size[1]

        height_scale = new_img_height / orig_img_height
        width_scale = new_img_width / orig_img_width

        new_bbox = [0] * 4
        new_bbox[0] = orig_bbox[0] * width_scale # x
        new_bbox[2] = orig_bbox[2] * width_scale # width
        new_bbox[1] = orig_bbox[1] * height_scale # y
        new_bbox[3] = orig_bbox[3] * height_scale # height

        return new_bbox

   def process_dataset(self, dataset_path):
        # Import unprocessed labels
        with open(os.path.join(dataset_path, "result.json"), 'r') as f:
            json_data = json.load(f)
        
        # Create images_processed directory
        processed_path = os.path.join(dataset_path, "images_preprocessed")
        os.makedirs(processed_path, exist_ok=True)
        
        # Copy JSON data
        processed_json_data = copy.deepcopy(json_data)

        # Dictionaries for tracking data properties
        id_mapping = {
            category['id']: {v: k for k, v in self.coco_id_to_name_dict.items()}[category['name'].lower()]
            for category in json_data['categories']
        }

        # Process images and update annotations
        for image in json_data["images"]:
            image_id = image['id']            
            # Process image
            file_name = image["file_name"]
            img_path = os.path.join(dataset_path, file_name)
            
            print(f"Processing {file_name}...")
            processed_img, rot_size = self._process_image(img_path)

            # Update image paths and sizes
            image["file_name"] = os.path.join("images_preprocessed", os.path.basename(image["file_name"]))
            image["width"] = rot_size[0]
            image["height"] = rot_size[1]
            save_path = os.path.join(processed_path, os.path.basename(image["file_name"]))
            processed_img.save(save_path)
        
        # Get all image sizes, store in dictionary and update sizes
        orig_img_sizes_dict = {}
        for image in json_data["images"]:
            # Store image sizes in dictionary
            orig_img_size = (image["height"], image["width"])
            img_id = image["id"]
            orig_img_sizes_dict[img_id] = orig_img_size
        
        for image in processed_json_data["images"]:
            # Update image sizes
            image["height"] = self.img_size[0]
            image["width"] = self.img_size[1]
            # Update image paths
            image["file_name"] = os.path.join("images_preprocessed", os.path.basename(image["file_name"]))
        
        
        for ann in processed_json_data["annotations"]:
            image_id = ann["image_id"]

            orig_img_size = orig_img_sizes_dict[image_id]
            orig_bbox = ann["bbox"]
            new_bbox = self.resize_bbox(orig_img_size, self.img_size, orig_bbox)
            # Update new bbox size
            ann["bbox"] = new_bbox
            
            # Update id
            orig_cat_id = ann["category_id"]
            coco_id = id_mapping[orig_cat_id]
            ann["category_id"] = coco_id

            if "area" in ann: del ann["area"]
        
        # Update category info
        for category in processed_json_data["categories"]:
            name = category["name"]
            if name in self.coco_name_to_id_dict.keys():
                category["id"] = self.coco_name_to_id_dict[name]
        
        # Save processed JSON
        json_path = os.path.join(dataset_path, "result_preprocessed.json")
        with open(json_path, 'w') as f:
            json.dump(processed_json_data, f, indent=4)


   def visualize_annotations(self, img_path, annotations, id_to_color=None):
        if id_to_color is None:
            id_to_color = {10: "green", 13: "red"}
        
        img = Image.open(img_path)
        draw = ImageDraw.Draw(img)
        
        for ann in annotations:
            bbox = ann["bbox"]
            color = id_to_color.get(ann["category_id"], "white")
            draw.rectangle([bbox[0], bbox[1], bbox[0] + bbox[2], bbox[1] + bbox[3]], 
                        outline=color, width=2)
        
        save_dir = Path.home() / "section_assets" / "vision_data"
        save_dir.mkdir(parents=True, exist_ok=True)
        img.save(save_dir / 'annotated_image.png', 'PNG')
        return img