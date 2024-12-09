import os
import shutil
from sklearn.model_selection import train_test_split

# Define your dataset directories
images_dir = 'Data'  # Folder containing images
labels_dir = 'Label'  # Folder containing labels
output_images_dir = 'opimg'
output_labels_dir = 'oplabel'

# Create output directories if they don't exist
os.makedirs(os.path.join(output_images_dir, 'train'), exist_ok=True)
os.makedirs(os.path.join(output_images_dir, 'val'), exist_ok=True)
os.makedirs(os.path.join(output_labels_dir, 'train'), exist_ok=True)
os.makedirs(os.path.join(output_labels_dir, 'val'), exist_ok=True)

# Get list of all image files
image_files = [f for f in os.listdir(images_dir) if os.path.isfile(os.path.join(images_dir, f))]
print("Image files found:", image_files)

# Extract base names (without extensions) for splitting
base_names = [os.path.splitext(f)[0] for f in image_files]
print("Base names extracted:", base_names)

# Split data into training and validation sets
train_files, val_files = train_test_split(base_names, test_size=0.3, random_state=42)
print("Training files:", train_files)
print("Validation files:", val_files)

# Helper function to copy files to destination folder
def copy_files(file_list, src_dir, dest_dir, is_image=True):
    for base_name in file_list:
        file_extension = '.jpg' if is_image else '.txt'  # Adjust extension if needed
        src_file_path = os.path.join(src_dir, base_name + file_extension)
        dest_file_path = os.path.join(dest_dir, base_name + file_extension)
        
        if os.path.exists(src_file_path):
            shutil.copy(src_file_path, dest_file_path)
            print(f"Copied {'image' if is_image else 'label'}: {src_file_path} to {dest_file_path}")
        else:
            print(f"{'Image' if is_image else 'Label'} not found: {src_file_path}")

# Copy images and labels for training and validation sets
print("Copying training images...")
copy_files(train_files, images_dir, os.path.join(output_images_dir, 'train'), is_image=True)

print("Copying validation images...")
copy_files(val_files, images_dir, os.path.join(output_images_dir, 'val'), is_image=True)

print("Copying training labels...")
copy_files(train_files, labels_dir, os.path.join(output_labels_dir, 'train'), is_image=False)

print("Copying validation labels...")
copy_files(val_files, labels_dir, os.path.join(output_labels_dir, 'val'), is_image=False)
