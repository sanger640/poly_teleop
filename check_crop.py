import cv2
import numpy as np
import pyrealsense2 as rs

# --- CONFIGURATION (Must match your training YAML) ---
# 1. Input Resolution 
# This is what you resize the raw camera stream to BEFORE cropping.
# Matches 'shape_meta.obs.agent_view_image' in your config.
INPUT_W, INPUT_H = 320, 240

# 2. Crop Resolution
# This is the actual input size to the Neural Network after augmentation.
# Matches 'policy.crop_shape' in your config.
# Config format is usually [Height, Width], so check carefully!
CROP_H, CROP_W = 216, 288 
# -----------------------------------------------------

def main():
    # 1. Setup RealSense
    print(f"Connecting to RealSense...")
    pipeline = rs.pipeline()
    config = rs.config()
    
    # We capture at 640x480 (standard) and resize down manually
    # to match exactly what the training script does.
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"Error connecting to camera: {e}")
        print("Check USB connection or if another script is using the camera.")
        return

    print("\n--- VISUAL DEBUGGER RUNNING ---")
    print(f"Resizing to: {INPUT_W}x{INPUT_H}")
    print(f"Center Crop: {CROP_W}x{CROP_H}")
    print("--------------------------------")
    print("GREEN BOX = What the Neural Network sees.")
    print("OUTSIDE   = Blind spot.")
    print("--------------------------------")
    print("Press 'q' to quit.")

    try:
        while True:
            # 2. Get Frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert to numpy
            frame = np.asanyarray(color_frame.get_data())

            # 3. Resize to Input Resolution (e.g. 320x240)
            # This simulates the first step of the data loader
            debug_img = cv2.resize(frame, (INPUT_W, INPUT_H))

            # 4. Calculate Center Crop (Inference Logic)
            # The code takes a center crop during evaluation
            start_y = (INPUT_H - CROP_H) // 2
            start_x = (INPUT_W - CROP_W) // 2
            end_y = start_y + CROP_H
            end_x = start_x + CROP_W

            # 5. Draw the Box
            # Green Rectangle (0, 255, 0) with thickness 2
            cv2.rectangle(debug_img, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
            
            # Optional: Darken the outside area to emphasize what is "lost"
            # (Create a mask)
            overlay = debug_img.copy()
            cv2.rectangle(overlay, (0, 0), (INPUT_W, INPUT_H), (0, 0, 0), -1) # Black out everything
            cv2.rectangle(overlay, (start_x, start_y), (end_x, end_y), (0, 0, 0), -1) # Clear center (inverted logic for masking is harder in pure cv2, simple blend is easier)
            
            # Simpler visual: Just show the box.
            
            # 6. Display
            # We scale it up 2x just so it's easier for you to see on your monitor
            display_img = cv2.resize(debug_img, (INPUT_W * 2, INPUT_H * 2), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Network Input Debugger", display_img)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()