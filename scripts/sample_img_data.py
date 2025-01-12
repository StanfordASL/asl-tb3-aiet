import os
import shutil

def make_folder(path, folder_name):
    try:
        folder_path = os.path.join(path, folder_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"{folder_name} folder created...")
        else:
            print(f"{folder_name} folder already exists...")
    except FileNotFoundError:
        print(f"Error: The path '{path}' does not exist.")
    except PermissionError:
        print(f"Error: Permission denied for accessing '{path}'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")    

def print_folder_names(path):
    try:
        entries = os.listdir(path)

        print("Folders:")
        for entry in entries:
            entry_path = os.path.join(path, entry)
            if os.path.isdir(entry_path):
                print(f"{entry_path}")
    except FileNotFoundError:
        print(f"Error: The path '{path}' does not exist.")
    except PermissionError:
        print(f"Error: Permission denied for accessing '{path}'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def count_driving_data_imgs(path):
    try:
        n_imgs = 0
        entries = os.listdir(path)
        for entry in entries:
            entry_path = os.path.join(path, entry)
            if os.path.isdir(entry_path):
                if entry.split("_")[0] == "demo":
                    imgs_path = os.path.join(entry_path, "images")
                    imgs = os.listdir(imgs_path)
                    n_imgs += len(imgs)
        return n_imgs

    except FileNotFoundError:
        print(f"Error: The path '{path}' does not exist.")
    except PermissionError:
        print(f"Error: Permission denied for accessing '{path}'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def get_integer(prompt):
    while True:
        try:
            # Prompt the user for input
            user_input = int(input(prompt))
            return user_input
        except ValueError:
            print("Invalid input. Please enter an integer.")

def move_sample_imgs(origin_path, target_path, spacing):
    try:
        # Validate inputs
        if not os.path.exists(origin_path) or not os.path.isdir(origin_path):
            raise FileNotFoundError(f"The origin path '{origin_path}' does not exist or is not a directory.")
        
        if not os.path.exists(target_path) or not os.path.isdir(target_path):
            raise FileNotFoundError(f"The target path '{target_path}' does not exist or is not a directory.")
        
        if not isinstance(spacing, int) or spacing <= 0:
            raise ValueError("Spacing must be a positive integer.")

        # List entries in the origin path
        entries = os.listdir(origin_path)
        n_copied = 0

        for entry in entries:
            entry_path = os.path.join(origin_path, entry)
            if os.path.isdir(entry_path) and entry.split("_")[0] == "demo":
                imgs_path = os.path.join(entry_path, "images")
                if not os.path.exists(imgs_path) or not os.path.isdir(imgs_path):
                    print(f"Warning: Images path '{imgs_path}' does not exist or is not a directory.")
                    continue

                imgs = [img for img in os.listdir(imgs_path) if os.path.isfile(os.path.join(imgs_path, img))]
                for count, img in enumerate(imgs):
                    if count % spacing == 0:
                        img_path = os.path.join(imgs_path, img)
                        shutil.copy(img_path, target_path)
                        n_copied += 1

        print(f"Copied {n_copied} images to {target_path}...")

    except FileNotFoundError as e:
        print(f"Error: {e}")
    except PermissionError as e:
        print(f"Permission Error: {e}")
    except ValueError as e:
        print(f"Value Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


if __name__ == "__main__":
    target_path = "~/section_assets/driving_data"
    full_path = os.path.expanduser(target_path)
    sample_folder = "imgs_sample"
    make_folder(full_path, sample_folder)
    print_folder_names(full_path)
    n_imgs = count_driving_data_imgs(full_path)
    print(f"{n_imgs} images found...")
    spacing = get_integer("Enter sample spacing: ")
    print(f"User entered: {spacing}")
    sample_path = os.path.join(full_path, sample_folder)
    move_sample_imgs(full_path, sample_path, spacing)

