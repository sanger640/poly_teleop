import cv2
import numpy as np
import pyrealsense2 as rs
import os
import glob

# --- CONFIGURATION ---
# Path to the frames of your first episode
EPISODE_PATH = "episodes/6/rgb_frames" 

# Serial number of the camera you want to check (e.g., Camera 1)
CAMERA_SERIAL = "215222078938" 

# Target Resolution (Match your training, usually 320x240)
TARGET_W, TARGET_H = 320, 240
# ---------------------

def main():
    # 1. Find the first frame of the episode
    print(f"Looking for frames in {EPISODE_PATH}...")
    
    # Get all cam1 images and sort them to find the first one
    search_pattern = os.path.join(EPISODE_PATH, "cam1_*.png")
    files = sorted(glob.glob(search_pattern))
    
    if not files:
        print(f"❌ No 'cam1_*.png' files found in {EPISODE_PATH}")
        return
        
    first_frame_path = files[0]
    print(f"✅ Loaded Reference: {os.path.basename(first_frame_path)}")
    
    # 2. Load Reference Image
    ref_img = cv2.imread(first_frame_path)
    if ref_img is None:
        print("❌ Failed to load image file.")
        return
    
    # Ensure reference is the correct size
    ref_img = cv2.resize(ref_img, (TARGET_W, TARGET_H))

    # 3. Start Live Camera
    print("Starting Live Camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(CAMERA_SERIAL)
    # Request standard resolution
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"❌ Error starting camera: {e}")
        return

    print("\n--- INSTRUCTIONS ---")
    print("1. Look at the window.")
    print("2. 'Double Vision' = Camera has moved.")
    print("3. Nudge your physical camera until the ghost image aligns perfectly.")
    print("4. Press 'Q' to quit.")
    print("--------------------")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue
            
            # Live Image
            live_img = np.asanyarray(color_frame.get_data())
            # Resize Live to match Reference (320x240)
            live_img = cv2.resize(live_img, (TARGET_W, TARGET_H))
            
            # 4. Create "Ghost" Overlay (50% blend)
            # Weighted sum: src1 * alpha + src2 * beta + gamma
            blended = cv2.addWeighted(live_img, 0.5, ref_img, 0.5, 0)
            
            # Scale up to 640x480 for easier viewing on your monitor
            display = cv2.resize(blended, (640, 480), interpolation=cv2.INTER_NEAREST)
            
            # Add text
            cv2.putText(display, "GREEN/GHOST = REFERENCE (OLD)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, "CLEAR = LIVE (NOW)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("Drift Check - Overlay", display)
            
            # Toggle viewing modes (Optional feature)
            # You can also show them side-by-side if you prefer
            # combined = np.hstack((live_img, ref_img))
            # cv2.imshow("Side by Side", combined)
            
            if cv2.waitKey(1) == ord('q'): 
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()