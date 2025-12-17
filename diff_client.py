import zmq
import numpy as np
import cv2
import pyrealsense2 as rs
import time
import torch
from scipy.spatial.transform import Rotation as R
from polymetis import RobotInterface, GripperInterface
from collections import deque


# --- CONFIG ---
GPU_SERVER_IP = "129.97.71.51" # CHANGE THIS to your GPU Desktop IP
PORT = 5555
ROBOT_IP = "129.97.71.27"         # IP of the Franka Panda (usually localhost if running on NUC)
GRIPPER_IP = "129.97.71.27"
IMG_W, IMG_H = 320, 240
CONTROL_HZ = 10
# --------------

def rot6d_to_quat(rot6d):
    """
    Converts 6D rotation (first 2 cols of rot matrix) back to Quaternion (x,y,z,w).
    Input: numpy array (6,)
    """
    # 1. Reshape to columns
    c1 = rot6d[:3]
    c2 = rot6d[3:]
    
    # 2. Gram-Schmidt Orthogonalization
    # Normalize col1
    c1 = c1 / np.linalg.norm(c1)
    # Get col3 (cross product)
    c3 = np.cross(c1, c2)
    c3 = c3 / np.linalg.norm(c3)
    # Recompute col2 to ensure orthogonality
    c2 = np.cross(c3, c1)
    
    # 3. Stack to get 3x3 Matrix
    matrix = np.stack([c1, c2, c3], axis=1)
    
    # 4. Convert to Quat (Scipy uses x,y,z,w)
    return R.from_matrix(matrix).as_quat()

def quat_to_rot6d(quat):
    """
    Converts Quaternion (x,y,z,w) to 6D Rotation.
    Input: numpy array (4,)
    """
    mat = R.from_quat(quat).as_matrix() # 3x3
    # Take first two columns and flatten
    return mat[:, :2].flatten() # (6,)

class RealSense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
        self.pipeline.start(config)
    
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        img = np.asanyarray(color_frame.get_data())
        # Resize & Convert BGR->RGB
        img = cv2.resize(img, (IMG_W, IMG_H))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Return format: (C, H, W)
        return np.transpose(img, (2, 0, 1))

class DiffusionClient:
    def __init__(self):
        print(f"Connecting to Polymetis at {ROBOT_IP}...")
        self.robot = RobotInterface(ip_address=ROBOT_IP)
        self.gripper = GripperInterface(ip_address=ROBOT_IP)
        self.prev_target_pos = None
        self.initial_robot_rot = None
        self.gripper_is_closed = False
        # Go home and start impedance mode (compliant mode)
        # self.robot.go_home() 
        self.Kx = torch.Tensor([750, 750, 750, 15, 15, 15])
        self.Kxd = torch.Tensor([37, 37, 37, 2, 2, 2])
        self.robot.start_cartesian_impedance(Kx=self.Kx, Kxd=self.Kxd)        
        print("Robot Connected & In Impedance Mode.")

    def get_state(self):
        # 1. Get Pose (Pos + Quat)
        pos, quat = self.robot.get_ee_pose()
        pos = pos.numpy()
        quat = quat.numpy() # [x, y, z, w]

        if self.initial_robot_rot is None:
            self.initial_robot_rot = quat
        # 2. Get Gripper State (Normalize to -1 or 1)
        # Assuming max width is ~0.08m
        grip_state = [1.0] if self.gripper_is_closed else [-1.0] # 1=Closed, -1=Open
        
        # # 3. Convert Quat -> Rot6D
        # rot6d = quat_to_rot6d(quat)
        
        # 4. Concat: [3 pos, 6 rot, 1 grip]
        return np.concatenate([pos, grip_state]).astype(np.float32)
        
    def execute(self, action):
        """
        Action shape (10,): [x,y,z, r1..r6, grip]
        """
        # 1. Unpack
        target_pos = torch.Tensor(action[:3])
        # alpha = 0.3 
        
        # if self.prev_target_pos is not None:
        #     smoothed_pos = (alpha * target_pos) + ((1 - alpha) * self.prev_target_pos)
        #     target_pos = smoothed_pos
            
        # self.prev_target_pos = target_pos
        # rot6d = action[3:9]
        grip_cmd = action[3]
        
        # 2. Convert Rot6D -> Quat
        # target_quat = torch.Tensor(rot6d_to_quat(rot6d))
        # print("Target Quat")
        # print(rot6d_to_quat(rot6d))
        # 3. Send Move Command (Async update to impedance controller)
        self.robot.update_desired_ee_pose(position=target_pos, orientation=torch.Tensor(self.initial_robot_rot))
        # self.prev_target_pos = target_pos
        # 4. Handle Gripper (Thresholding)
        # If model outputs > 0.5, assume it wants to close
        # print("grip cmd")
        # if grip_cmd >-0.7 and grip_cmd < 0.7:
        #     print("Uncertain")
        #     print(grip_cmd)
        if grip_cmd > 0.9:
            # Only send command if not already grasping to avoid spamming
            if not self.gripper_is_closed:
                self.gripper.grasp(speed=0.05, force=0.1)
                # print("Gripper Closed")
                self.gripper_is_closed = True
        elif grip_cmd < -0.9:
            if self.gripper_is_closed:
                self.gripper.stop()
                self.gripper.goto(width=0.25, speed=0.05, force=0.1)
                # print("Gripper Open")
                self.gripper_is_closed = False
        else:
            pass

# def main():
#     # 1. Setup ZMQ
#     context = zmq.Context()
#     socket = context.socket(zmq.REQ)
#     socket.connect(f"tcp://{GPU_SERVER_IP}:{PORT}")
#     print(f"Connected to GPU Server at {GPU_SERVER_IP}")

#     # 2. Hardware
#     camera = RealSense()
#     robot = DiffusionClient()
    
#     # 3. Buffers
#     obs_buffer = deque(maxlen=2) # n_obs_steps=2

#     try:
#         while True:
#             cycle_start = time.time()
            
#             # --- A. Capture ---
#             img = camera.get_frame()
#             state = robot.get_state()
#             obs_buffer.append({'img': img, 'state': state})
            
#             # Wait for buffer fill
#             if len(obs_buffer) < 2:
#                 time.sleep(0.1)
#                 continue

#             # --- B. Send to GPU ---
#             batch_img = np.stack([x['img'] for x in obs_buffer])
#             batch_state = np.stack([x['state'] for x in obs_buffer])
            
#             socket.send_pyobj({
#                 'img': batch_img.tobytes(),
#                 'state': batch_state.tobytes(),
#                 'shape': (IMG_H, IMG_W)
#             })
            
#             # --- C. Receive Action ---
#             response = socket.recv_pyobj()
#             actions = response['action'] # Shape (8, 10)
#             # print("Diffusion Action")
#             # print(actions[0])
#             # print("Current EE Pose")
#             # print(robot.robot.get_ee_pose())
#             # --- D. Execute ---
#             # IMPORTANT: We only execute the FIRST action to stay reactive at 10Hz
#             # The model gave us 8 steps, but we discard 7 and re-plan next frame.
#             robot.execute(actions[0])
            
#             # # --- E. Sleep ---
#             # elapsed = time.time() - cycle_start
#             # time.sleep(max(0, (1/CONTROL_HZ) - elapsed))

#     except KeyboardInterrupt:
#         print("Stopping...")
#         robot.robot.terminate_current_policy()

def main():
    # 1. Setup ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{GPU_SERVER_IP}:{PORT}")
    print(f"Connected to GPU Server at {GPU_SERVER_IP}")

    # 2. Hardware
    camera = RealSense()
    robot = DiffusionClient()
    
    # 3. Buffers
    obs_buffer = deque(maxlen=2) # n_obs_steps=2

    try:
        while True:
            # --- A. RE-FILL BUFFER (Critical Step) ---
            # Since we just spent ~0.8s executing actions, our old buffer is stale.
            # We need to grab 2 fresh frames to give the model an accurate "Now".
            obs_buffer.clear()
            
            # Quickly fill buffer (takes ~0.2s for 2 frames at 10Hz)
            # This ensures the model sees specific history [t-1, t]
            while len(obs_buffer) < 2:
                img = camera.get_frame()
                state = robot.get_state()
                obs_buffer.append({'img': img, 'state': state})
                # Slight sleep to respect physical time between frames if camera is fast
                # But RealSense .wait_for_frames() usually handles the timing.

            # --- B. INFERENCE ---
            print("Requesting new trajectory...")
            batch_img = np.stack([x['img'] for x in obs_buffer])
            batch_state = np.stack([x['state'] for x in obs_buffer])
            
            socket.send_pyobj({
                'img': batch_img.tobytes(),
                'state': batch_state.tobytes(),
                'shape': (IMG_H, IMG_W)
            })
            
            # Wait for GPU reply
            response = socket.recv_pyobj()
            actions = response['action'] # Shape (8, 4)
            
            print(f"Received {len(actions)} steps. Executing chunk...")

            # --- C. EXECUTION LOOP (Action Chunking) ---
            # Execute all 8 steps blindly (Open-Loop execution)
            for i, action in enumerate(actions):
                step_start = time.time()
                
                # 1. Send Command
                robot.execute(action)
                
                # 2. Sleep to maintain 10Hz Control Rate
                # elapsed = time.time() - step_start
                # sleep_time = max(0, (1.0 / CONTROL_HZ) - elapsed)
                # time.sleep(sleep_time)
                
            # After this loop finishes, we go back to top -> Refill Buffer -> Re-plan

    except KeyboardInterrupt:
        print("Stopping...")
        robot.robot.terminate_current_policy()


if __name__ == "__main__":
    main()