import paramiko
import os
from pathlib import Path
import sys
from preprocess_data import CVPreProcessManager
import subprocess
import argparse
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload, MediaIoBaseDownload
import json

#TODO: Fill in for rest of robots
"""
Laptop ID -> Robot name pairing. 
See https://shorturl.at/Ab1ib for current pairing list
"""
ROBOT_IDs = {
   "00" : "lucy",
   "01" : "rudi",
   "02" : "judy",
   "03" : "harlan",
   "04" : "orwell",
   "05" : "astro",
   "06" : "teddy",
   "07" : "asteroid",
   "08" : "petunia",
   "09" : "marcia",
   "10" : "montague",
   "11" : "macc",
   "12" : "henry",
   "13" : "rosie",
   "14" : "george",
}

class CVWorkflowManager:
    def __init__(self, robot_id=None, verbose=False):
        self.verbose = verbose
        if robot_id is None:
            laptop_id = os.environ.get('LAPTOP_ID')
            if laptop_id is None:
                raise ValueError("robot_id not specified and LAPTOP_ID environment variable not found")
            robot_id = laptop_id.split('-')[-1]

        self.robot_id = robot_id.zfill(2)
        self.robot_name = ROBOT_IDs[self.robot_id]
        self.checkpoint_name = "finetuned_ssd_model"

        # Set up paths
        self.base_dir = Path.home() / "section_assets"
        self.local_data_dir = self.base_dir / "vision_data"
        self.model_path = self.base_dir / f"{self.checkpoint_name}.pkl"
        self.processed_data_path = self.local_data_dir / "images_preprocessed"
        self.autonomy_dir = Path.home() / "autonomy_ws" / "src" / "asl-tb3-aiet" / "scripts"
        
        # Google Drive paths
        self.drive_base_path = "AIET/perception"
        self.robot_folder = f"robot_{self.robot_id}"
        self.notebook_name = "cv_finetune.ipynb"

        # Login details
        self.hostname = f"{self.robot_name}.local"
        self.uname = "aa274"
        self.password = "aa274"
        self.remote_path = f'/home/aa274/section_assets/{self.checkpoint_name}.pkl'

        self.debug_print("\nConfiguration:")
        self.debug_print(f"Robot ID: {self.robot_id}")
        self.debug_print(f"Base path: {self.drive_base_path}")
        self.debug_print(f"Robot folder: {self.robot_folder}")
        self.debug_print(f"Looking for notebook: {self.notebook_name}")
    
    def debug_print(self, *args, **kwargs):
        """Print only if verbose mode is enabled"""
        if self.verbose:
            print(*args, **kwargs)

    def send_file(self, local_path, remote_path):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        ssh.connect(self.hostname, username=self.uname, password=self.password)
        # Remove file and create directory
        ssh.exec_command('mkdir -p /home/aa274/section_assets')

        sftp = ssh.open_sftp()
        sftp.put(local_path, self.remote_path)
        sftp.close()
        ssh.close()

        return True
    
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
        
    def download_model(self, service):
        """Download trained model from Drive"""
        folder_path = f"{self.drive_base_path}/{self.robot_folder}"
        folder_id = self.get_or_create_folder(service, folder_path)
        
        query = f"name='{self.checkpoint_name}.pkl' and '{folder_id}' in parents"
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
    
    def get_folder_url(self, service):
        """Get the Google Drive web URL for the robot's folder"""
        folder_path = f"{self.drive_base_path}/{self.robot_folder}"
        self.debug_print(f"Getting URL for folder: {folder_path}")
        
        try:
            folder_id = self.get_or_create_folder(service, folder_path)
            url = f"https://drive.google.com/drive/folders/{folder_id}"
            self.debug_print(f"Found folder ID: {folder_id}")
            return url
        except Exception as e:
            print(f"Could not find folder: {folder_path}")
            self.debug_print(f"Error: {str(e)}")
            return None
    
    def preprocess_data(self):
        """Run the preprocessing script if processed data doesn't exist"""
        if self.processed_data_path.exists():
            print("Preprocessed data already exists, skipping preprocessing step...")
            return

        print("Running CV Preprocessing...")
        try:
            CVmanager = CVPreProcessManager()
            CVmanager.process_dataset(self.local_data_dir)
        except subprocess.CalledProcessError as e:
            print(f"Error running CV PreProcessing: {e}")
            raise
    
    def run_workflow(self):
        """Execute the complete workflow"""
        try:
            print(f"Starting CV workflow for Robot {self.robot_id}")

            # print("1. Preprocessing data...")
            # self.preprocess_data()
            
            # print("2. Setting up Google Drive connection...")
            print("1. Setting up Google Drive connection...")
            service = self.setup_service_account()
            
            if self.verbose:
                self.check_drive_access(service)
            
            # print("3. Upload processed data to Drive...")
            print("2. Upload processed data to Drive...")
            folder_url = self.get_folder_url(service)
            if folder_url:
                # print(f"\nPlease upload dataset to drive using this link:\n{folder_url}")
                # print("\nMake sure to:")
                # print("1. Upload images_preprocessed/ data folder")
                # print("2. Upload result_preprocessed.json file")
                # input("\nPress Enter once training is complete...")
                
                # print("4. Downloading trained model...")
                print("3. Downloading trained model...")
                if self.download_model(service):
                    print("Model downloaded successfully!\n")
                else:
                    print("Failed to download model. Please check if training completed successfully.")
            
            # print("5. Upload computer vision model to robot")
            print("4. Upload computer vision model to robot")
            print("\nMake sure to:")
            print("1. Turn robot on.")
            print(f"2. Ensure robot is on network (e.g., ping {self.robot_name}.local)")
            input("\nPress Enter once robot is online...")
            if self.send_file(self.model_path, self.remote_path):
                print("Model successfully sent to robot!")
            else:
                print("Failed to send model.")
        
        except Exception as e:
            print(f"Error in workflow: {e}")
            sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Manage CV training workflow")
    parser.add_argument('--robot-id', type=str, help='Robot ID (e.g., 01, 02)', 
                       default=None)
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose debug output')
    args = parser.parse_args()
    
    manager = CVWorkflowManager(robot_id=args.robot_id, verbose=args.verbose)
    manager.run_workflow()
#    CVWorkflowManager.send_file(local_path=os.path.expanduser('~/section_assets/finetuned_ssd_model.pth'),
#                                remote_path='/home/aa274/section_assets/finetuned_ssd_model.pth')