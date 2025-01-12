#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path
import pickle
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload, MediaIoBaseDownload
import json
from preprocess_data import ILPreProcessManager

class ILWorkflowManager:
    def __init__(self, robot_id=None, verbose=False):
        self.verbose = verbose
        if robot_id is None:
            laptop_id = os.environ.get('LAPTOP_ID')
            if laptop_id is None:
                raise ValueError("robot_id not specified and LAPTOP_ID environment variable not found")
            robot_id = laptop_id.split('-')[-1]

        self.robot_id = robot_id.zfill(2)
        self.checkpoint_name = "IL_model_checkpoint"

        # Set up paths
        self.base_dir = Path.home() / "section_assets"
        self.local_data_dir = self.base_dir / "driving_data"
        self.processed_data_path = self.local_data_dir / "processed_data.pkl"
        self.model_path = self.base_dir / f"{self.checkpoint_name}.pth"
        self.autonomy_dir = Path.home() / "autonomy_ws" / "src" / "asl-tb3-autonomy" / "scripts"

        # Ensure directories exist
        self.local_data_dir.mkdir(parents=True, exist_ok=True)
        
        # Google Drive paths
        self.drive_base_path = "AIET/imitation_learning"
        self.robot_folder = f"robot_{self.robot_id}"
        self.notebook_name = "IL_training.ipynb"

        self.debug_print("\nConfiguration:")
        self.debug_print(f"Robot ID: {self.robot_id}")
        self.debug_print(f"Base path: {self.drive_base_path}")
        self.debug_print(f"Robot folder: {self.robot_folder}")
        self.debug_print(f"Looking for notebook: {self.notebook_name}")

    def debug_print(self, *args, **kwargs):
        """Print only if verbose mode is enabled"""
        if self.verbose:
            print(*args, **kwargs)

    def setup_service_account(self):
        """Setup Google Drive API using service account"""
        SCOPES = ['https://www.googleapis.com/auth/drive']
        
        service_account_path = self.base_dir / 'service-account.json'
        if not service_account_path.exists():
            service_account_path = Path.cwd() / 'service-account.json'
        
        if not service_account_path.exists():
            raise FileNotFoundError(
                "service-account.json not found! Please download service account "
                "credentials and save as 'service-account.json'"
            )
        
        credentials = service_account.Credentials.from_service_account_file(
            str(service_account_path), scopes=SCOPES)
        
        # Service account details only in verbose mode
        if self.verbose:
            with open(service_account_path) as f:
                sa_info = json.load(f)
                self.debug_print("\nService Account Details:")
                self.debug_print(f"Email: {sa_info['client_email']}")
                self.debug_print(f"Project: {sa_info['project_id']}")
        
        service = build('drive', 'v3', credentials=credentials)
        
        # Check shared drive access
        self.debug_print("\nSearching for shared folder...")
        response = service.files().list(
            q="name='AIET' and 'tps1.aiet@gmail.com' in owners",
            spaces='drive',
            fields='files(id, name, owners)',
            supportsAllDrives=True,
            includeItemsFromAllDrives=True
        ).execute()
        
        files = response.get('files', [])
        if not files:
            print("Could not find AIET folder owned by tps1.aiet@gmail.com")
            print("Please ensure the folder is shared with the service account")
        else:
            self.debug_print(f"Found shared folder: {files[0]['name']}")
            self.debug_print(f"Owner: {files[0]['owners'][0]['emailAddress']}")
        
        return service

    def get_or_create_folder(self, service, folder_path):
        """Create nested folder structure in Drive"""
        root_folder = folder_path.split('/')[0]
        query = f"name='{root_folder}' and mimeType='application/vnd.google-apps.folder'"
        results = service.files().list(
            q=query,
            spaces='drive',
            fields='files(id, name)',
            supportsAllDrives=True,
            includeItemsFromAllDrives=True
        ).execute()
        files = results.get('files', [])
        
        if not files:
            raise FileNotFoundError(f"Root folder '{root_folder}' not found in Drive")
        
        parent_id = files[0]['id']
        remaining_path = folder_path.split('/')[1:]
        
        for folder_name in remaining_path:
            if not folder_name:
                continue
            
            query = f"name='{folder_name}' and '{parent_id}' in parents and mimeType='application/vnd.google-apps.folder'"
            results = service.files().list(
                q=query,
                spaces='drive',
                fields='files(id, name)',
                supportsAllDrives=True,
                includeItemsFromAllDrives=True
            ).execute()
            files = results.get('files', [])
            
            if files:
                parent_id = files[0]['id']
                self.debug_print(f"Found existing folder: {folder_name}")
            else:
                self.debug_print(f"Creating new folder: {folder_name}")
                folder_metadata = {
                    'name': folder_name,
                    'mimeType': 'application/vnd.google-apps.folder',
                    'parents': [parent_id]
                }
                folder = service.files().create(body=folder_metadata, fields='id').execute()
                parent_id = folder['id']
        
        return parent_id

    def preprocess_data(self):
        """Run the preprocessing script if processed data doesn't exist"""
        if self.processed_data_path.exists():
            print("Preprocessed data already exists, skipping preprocessing step...")
            return

        print("Running IL Preprocessing...")
        try:
            ILmanager = ILPreProcessManager()
            ILmanager.process_dataset(self.local_data_dir)
        except subprocess.CalledProcessError as e:
            print(f"Error running IL PreProcessing: {e}")
            raise

    def upload_to_drive(self, service):
        """Upload processed data to specific robot folder"""
        folder_path = f"{self.drive_base_path}/{self.robot_folder}"
        self.debug_print(f"Creating/finding folder structure: {folder_path}")
        
        try:
            folder_id = self.get_or_create_folder(service, folder_path)
            self.debug_print(f"Successfully got folder ID: {folder_id}")
            
            file_metadata = {
                'name': 'processed_data.pkl',
                'parents': [folder_id]
            }
            
            self.debug_print(f"Uploading file from: {self.processed_data_path}")
            media = MediaFileUpload(self.processed_data_path, 
                                mimetype='application/octet-stream',
                                resumable=True)
            
            query = f"name='processed_data.pkl' and '{folder_id}' in parents"
            results = service.files().list(q=query, fields='files(id)').execute()
            files = results.get('files', [])
            
            if files:
                self.debug_print(f"Updating existing file with ID: {files[0]['id']}")
                file = service.files().update(
                    fileId=files[0]['id'],
                    media_body=media).execute()
            else:
                self.debug_print("Creating new file")
                file = service.files().create(
                    body=file_metadata,
                    media_body=media,
                    fields='id').execute()
            
            print("Upload successful!")
            self.debug_print(f"File ID: {file.get('id')}")
            return file.get('id')
            
        except Exception as e:
            print(f"Error during upload: {str(e)}")
            raise

    def check_drive_access(self, service):
        """Debug helper to check drive access"""
        if not self.verbose:
            return

        self.debug_print("\nChecking Drive access:")
        try:
            about = service.about().get(fields="user").execute()
            self.debug_print(f"Authenticated as: {about['user']['emailAddress']}")
            
            query = f"name='AIET' and mimeType='application/vnd.google-apps.folder'"
            results = service.files().list(
                q=query,
                spaces='drive',
                fields='files(id, name, owners, permissions)',
                supportsAllDrives=True,
                includeItemsFromAllDrives=True
            ).execute()
            
            files = results.get('files', [])
            if files:
                self.debug_print(f"Found AIET folder: {files[0]['id']}")
                return files[0]['id']
            else:
                self.debug_print("Could not find AIET folder")
                return None
        except Exception as e:
            self.debug_print(f"Error checking drive access: {str(e)}")
            return None

    def get_notebook_url(self, service):
        """Get the URL for this robot's Colab notebook"""
        folder_path = f"{self.drive_base_path}/{self.robot_folder}"
        self.debug_print(f"\nSearching for notebook in: {folder_path}")
        
        try:
            folder_id = self.get_or_create_folder(service, folder_path)
            query = f"name='{self.notebook_name}' and '{folder_id}' in parents"
            results = service.files().list(
                q=query,
                spaces='drive',
                fields='files(id, name, webViewLink)',
                supportsAllDrives=True,
                includeItemsFromAllDrives=True
            ).execute()
            
            files = results.get('files', [])
            if files and 'webViewLink' in files[0]:
                self.debug_print(f"Found notebook: {files[0]['name']}")
                return files[0]['webViewLink']
            
            print(f"Could not find notebook {self.notebook_name}")
            print(f"Please ensure the notebook exists in the correct folder and is shared with the service account")
            return None
            
        except Exception as e:
            print(f"Error finding notebook: {str(e)}")
            return None

    def download_model(self, service):
        """Download trained model from Drive"""
        folder_path = f"{self.drive_base_path}/{self.robot_folder}"
        folder_id = self.get_or_create_folder(service, folder_path)
        
        query = f"name='{self.checkpoint_name}.pth' and '{folder_id}' in parents"
        results = service.files().list(q=query, fields='files(id)').execute()
        files = results.get('files', [])
        
        if not files:
            print("Model not found in Drive. Has training completed?")
            return False
        
        self.debug_print(f"Found model file with ID: {files[0]['id']}")    
        request = service.files().get_media(fileId=files[0]['id'])
        with open(self.model_path, 'wb') as f:
            downloader = MediaIoBaseDownload(f, request)
            done = False
            while done is False:
                status, done = downloader.next_chunk()
                if self.verbose:
                    self.debug_print(f"Download progress: {int(status.progress() * 100)}%")
        return True

    def run_workflow(self):
        """Execute the complete workflow"""
        try:
            print(f"Starting workflow for Robot {self.robot_id}")
            
            print("1. Preprocessing data...")
            self.preprocess_data()
            
            if not self.processed_data_path.exists():
                raise FileNotFoundError("Preprocessing did not generate processed_data.pkl")
            
            print("2. Setting up Google Drive connection...")
            service = self.setup_service_account()
            
            if self.verbose:
                self.check_drive_access(service)
            
            print("3. Uploading processed data to Drive...")
            self.upload_to_drive(service)
            
            print("4. Getting Colab notebook URL...")
            notebook_url = self.get_notebook_url(service)
            if notebook_url:
                print(f"\nPlease open this Colab notebook to train:\n{notebook_url}")
                print("\nMake sure to:")
                print("1. Connect to a GPU runtime")
                print("2. Define neural network architecture")
                print("3. Run all cells")
                input("\nPress Enter once training is complete...")
                
                print("5. Downloading trained model...")
                if self.download_model(service):
                    print("Model downloaded successfully!")
                else:
                    print("Failed to download model. Please check if training completed successfully.")
            
        except Exception as e:
            print(f"Error in workflow: {e}")
            sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Manage IL training workflow")
    parser.add_argument('--robot-id', type=str, help='Robot ID (e.g., 01, 02)', 
                       default=None)
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose debug output')
    args = parser.parse_args()
    
    manager = ILWorkflowManager(robot_id=args.robot_id, verbose=args.verbose)
    manager.run_workflow()