from PIL import Image
import os

def crop_image_to_320x240(image_path, output_path):
    """
    Opens an image, crops the center 320x240 section from a 640x480 image,
    and saves the cropped image.
    
    Args:
        image_path (str): The file path to the input image (expected 640x480).
        output_path (str): The file path to save the cropped output image.
    """
    try:
        # 1. Open the image
        img = Image.open(image_path)
        
        # Verify the starting size (optional, but good practice)
        if img.size != (640, 480):
            print(f"⚠️ Warning: Input image size is {img.size}, not the expected 640x480.")

        # Define the target dimensions
        target_width = 320
        target_height = 240
        
        # Calculate the coordinates for a **center crop**
        # The coordinates are (left, upper, right, lower)
        
        # Calculate starting points (left/upper)
        left = (img.width - target_width) / 2
        top = (img.height - target_height) / 2
        
        # Calculate ending points (right/lower)
        right = (img.width + target_width) / 2
        bottom = (img.height + target_height) / 2
        
        # The box tuple must be integer values
        crop_box = (int(left), int(top), int(right), int(bottom))
        
        print(f"Input Image Size: {img.size}")
        print(f"Crop Box (left, upper, right, lower): {crop_box}")

        # 2. Crop the image
        cropped_img = img.crop(crop_box)
        
        # 3. Save the new image
        cropped_img.save(output_path)
        
        print(f"\n✅ Successfully cropped and saved image to: {output_path}")
        print(f"Output Image Size: {cropped_img.size}")

    except FileNotFoundError:
        print(f"❌ Error: Image file not found at {image_path}. Please check the path.")
    except Exception as e:
        print(f"An error occurred: {e}")

# --- Example Usage ---

# **IMPORTANT:** Change 'input_image.jpg' to the actual path of your 640x480 image.
# If you don't have one, you'll need to create a placeholder or use an existing one.
INPUT_FILE = '/app/episodes/6/rgb_frames/cam2_1764950095632.png' 
OUTPUT_FILE = 'output_image_320x240.jpg'

# Ensure the Pillow library is installed:
# pip install Pillow

# Note: The code assumes an input image named 'input_image.jpg' is in the same directory.

crop_image_to_320x240(INPUT_FILE, OUTPUT_FILE)