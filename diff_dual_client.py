import zmq
import numpy as np
import cv2
import pyrealsense2 as rs
import time
import torch
import threading
from scipy.spatial.transform import Rotation as R
from polymetis import RobotInterface, GripperInterface
from collections import deque

# --- CONFIGURATION ---
GPU_SERVER_IP = "129.97.71.51" # UPDATE THIS
PORT = 5555
ROBOT_IP = "129.97.71.27"

# SERIAL NUMBERS (Must match training order!)
CAM1_SERIAL = "215222078938" 
CAM2_SERIAL = "819612070440"

# Hardware Config
IMG_W, IMG_H = 320, 240
CONTROL_HZ = 10
# ---------------------
class DualRealSenseAsync:
    def __init__(self):
        self.running = False
        self.lock = threading.Lock()
        
        # Internal Queue to hold the last 2 frames (Observation Horizon)
        # We pre-fill it with zeros or wait for first frames
        self.obs_queue = deque(maxlen=2)
        
        # Hardware Config
        self.p1 = rs.pipeline()
        c1 = rs.config()
        c1.enable_device(CAM1_SERIAL)
        c1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.p2 = rs.pipeline()
        c2 = rs.config()
        c2.enable_device(CAM2_SERIAL)
        c2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.p1.start(c1)
        self.p2.start(c2)
        print("✅ Dual Cameras Started (Async Mode)")
        
        # Start the Background Thread
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()
        
        # Block until queue is full (warmup)
        print("Waiting for camera warmup...")
        while len(self.obs_queue) < 2:
            time.sleep(0.1)
        print("✅ Camera Ready")

    def _update_loop(self):
        """Background thread to fetch frames at ~10Hz (or hardware speed)"""
        last_time = time.time()
        interval = 1.0 / CONTROL_HZ
        
        while self.running:
            # 1. Fetch from Hardware (Blocking, likely 30Hz)
            # We fetch as fast as possible to clear hardware buffers
            f1 = self.p1.wait_for_frames()
            f2 = self.p2.wait_for_frames()
            
            # 2. Rate Limiting (Update queue only at 10Hz)
            # This decouples capture rate from storage rate
            now = time.time()
            if now - last_time < interval:
                continue
            last_time = now

            # 3. Process
            i1 = np.asanyarray(f1.get_color_frame().get_data())
            i2 = np.asanyarray(f2.get_color_frame().get_data())
            
            # Resize & RGB
            i1 = cv2.resize(i1, (IMG_W, IMG_H))
            i1 = cv2.cvtColor(i1, cv2.COLOR_BGR2RGB) # H, W, C
            
            i2 = cv2.resize(i2, (IMG_W, IMG_H))
            i2 = cv2.cvtColor(i2, cv2.COLOR_BGR2RGB)
            
            # Channel First (C, H, W)
            i1 = np.transpose(i1, (2, 0, 1))
            i2 = np.transpose(i2, (2, 0, 1))
            
            # 4. Update Queue (Thread Safe)
            with self.lock:
                self.obs_queue.append({'c1': i1, 'c2': i2})

    def get_observation_batch(self):
        with self.lock:
            if len(self.obs_queue) < 2:
                # If the queue isn't ready, return None or raise a custom error
                return None, None
            # print(self.obs_queue)
            snapshot = list(self.obs_queue)
        
        b_c1 = np.stack([x['c1'] for x in snapshot])
        b_c2 = np.stack([x['c2'] for x in snapshot])
        return b_c1, b_c2

    def stop(self):
        self.running = False
        self.thread.join()
        self.p1.stop()
        self.p2.stop()

class DualRealSense:
    def __init__(self):
        # Cam 1
        self.p1 = rs.pipeline()
        c1 = rs.config()
        c1.enable_device(CAM1_SERIAL)
        c1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Cam 2
        self.p2 = rs.pipeline()
        c2 = rs.config()
        c2.enable_device(CAM2_SERIAL)
        c2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.p1.start(c1)
        self.p2.start(c2)
        print("✅ Dual Cameras Started")

    def get_frames(self):
        # Synchronized wait (might slow down if one camera lags)
        f1 = self.p1.wait_for_frames()
        f2 = self.p2.wait_for_frames()
        
        i1 = np.asanyarray(f1.get_color_frame().get_data())
        i2 = np.asanyarray(f2.get_color_frame().get_data())
        
        # Resize & RGB
        i1 = cv2.resize(i1, (IMG_W, IMG_H))
        i1 = cv2.cvtColor(i1, cv2.COLOR_BGR2RGB)
        
        i2 = cv2.resize(i2, (IMG_W, IMG_H))
        i2 = cv2.cvtColor(i2, cv2.COLOR_BGR2RGB)
        
        # Channel First (C, H, W)
        i1 = np.transpose(i1, (2, 0, 1))
        i2 = np.transpose(i2, (2, 0, 1))
        
        return i1, i2

class DiffusionClient:
    def __init__(self):
        print(f"Connecting to Polymetis at {ROBOT_IP}...")
        self.robot = RobotInterface(ip_address=ROBOT_IP)
        self.gripper = GripperInterface(ip_address=ROBOT_IP)
        self.prev_target_pos = None
        self.initial_robot_rot = None
        self.gripper_is_closed = False
        self.Kx = torch.Tensor([750, 750, 750, 15, 15, 15])
        self.Kxd = torch.Tensor([37, 37, 37, 2, 2, 2])
        self.robot.start_cartesian_impedance(Kx=self.Kx, Kxd=self.Kxd)
        init_pos, _ = self.robot.get_ee_pose()
        self.last_cmd_pos = (init_pos.numpy())
        print("Robot Connected & In Impedance Mode.")

    def get_state(self):
        pos, quat = self.robot.get_ee_pose()
        pos = pos.numpy()
        quat = quat.numpy() # [x, y, z, w]

        if self.initial_robot_rot is None:
            self.initial_robot_rot = quat
        # 4D State: [x, y, z, grip_cmd]
        grip_state = [1.0] if self.gripper_is_closed else [-1.0] # 1=Closed, -1=Open

        return np.concatenate([self.last_cmd_pos, grip_state]).astype(np.float32)
    
    def get_state_batch(self):
        """Replicates state 2 times to match image horizon (or use history if preferred)"""
        # Note: If you want state history, you'd need a deque here too. 
        # For now, we replicate current state T times as is common if state is Markovian enough.
        
        pos, quat = self.robot.get_ee_pose()
        if self.initial_robot_rot is None:
            self.initial_robot_rot = quat.numpy()
            
        grip_val = 1.0 if self.gripper_is_closed else -1.0
        
        # Construct single state vector
        # Using last_cmd_pos to fix "Sag/Lag" mismatch
        state_vec = np.concatenate([self.last_cmd_pos, [grip_val]]).astype(np.float32)
        
        # Stack 2 times: Shape (2, D)
        return np.stack([state_vec, state_vec])
        
    def execute(self, action):
        # Action: [x, y, z, grip]
        target_pos = torch.Tensor(action[:3])
        # Fixed Down Orientation
        # target_quat = torch.Tensor([1.0, 0.0, 0.0, 0.0]) 
        grip_cmd = action[3]
        self.last_cmd_pos = action[:3]
        self.robot.update_desired_ee_pose(position=target_pos, orientation=torch.Tensor(self.initial_robot_rot))
        
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

import numpy as np

class TemporalEnsembler:
    def __init__(self, horizon, action_dim, max_weight=1.0, min_weight=0.0):
        """
        Exponentially weighted temporal ensembling.
        weights decay from max_weight (at t=0) to min_weight (at t=H).
        """
        self.horizon = horizon
        self.action_dim = action_dim
        self.instart = False
        
        # 1. Create the Weight Vector (Exponential Decay)
        # We calculate 'k' such that: exp(-k * horizon) = min_weight
        # This ensures the last step has very low trust.
        decay_rate = -np.log(min_weight) / horizon
        
        # Shape (Horizon, 1) -> e.g. [1.0, 0.8, 0.64, ..., 0.01]
        time_steps = np.arange(horizon)
        self.weights = (max_weight * np.exp(-decay_rate * time_steps)).reshape(-1, 1)
        
        # Buffers
        self.accumulated_actions = np.zeros((horizon, action_dim))
        self.accumulated_weights = np.zeros((horizon, 1))

    def update(self, new_action_sequence):
        """
        Input: new_action_sequence (Horizon, Dim)
        """
        
        # 2. Weighted Accumulation
        # Multiply the new chunk by our weight profile before adding
        weighted_seq = new_action_sequence * self.weights
        
        self.accumulated_actions += weighted_seq
        self.accumulated_weights += self.weights

        # 3. Compute Weighted Average
        # Avoid division by zero (though mathematically unlikely with exp weights)
        scale = np.copy(self.accumulated_weights)
        scale[scale == 0] = 1.0 
        
        smooth_actions = self.accumulated_actions[0] / scale[0]

        # 4. Time Shift
        # Shift Actions
        self.accumulated_actions[:-1] = self.accumulated_actions[1:]
        self.accumulated_actions[-1] = 0
        
        # Shift Weights
        self.accumulated_weights[:-1] = self.accumulated_weights[1:]
        self.accumulated_weights[-1] = 0

        return smooth_actions

    def reset(self):
        self.accumulated_actions.fill(0)
        self.accumulated_weights.fill(0)

class AsyncInference:
    def __init__(self, cameras, robot):
        self.cameras = cameras
        self.robot = robot
        self.lock = threading.Lock()
        
        # ZMQ Setup
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{GPU_SERVER_IP}:{PORT}")
        
        # State
        self.latest_actions = None
        self.running = True
        
        # Start Thread
        self.thread = threading.Thread(target=self._inference_loop, daemon=True)
        self.thread.start()
        print("✅ Inference Thread Started")

    def _inference_loop(self):
        while self.running:
            # 1. Gather Data (Non-blocking check)
            b_c1, b_c2 = self.cameras.get_observation_batch()
            if b_c1 is None:
                time.sleep(0.005)
                continue
                
            b_s = self.robot.get_state_batch()

            # 2. Network I/O (The Slow Part)
            # We do this in the background so the robot doesn't freeze
            try:
                self.socket.send_pyobj({
                    'cam1': b_c1.tobytes(),
                    'cam2': b_c2.tobytes(),
                    'state': b_s.tobytes(),
                    'shape': (240, 320)
                })
                
                response = self.socket.recv_pyobj()
                new_actions = response['action'] # (16, 4)
                
                # 3. Hot-Swap the Plan
                with self.lock:
                    self.latest_actions = new_actions
                    
            except Exception as e:
                print(f"Inference Error: {e}")

    def get_latest_plan(self):
        with self.lock:
            if self.latest_actions is None:
                return None
            return self.latest_actions.copy()

    def stop(self):
        self.running = False
        self.thread.join()

def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{GPU_SERVER_IP}:{PORT}")
    print(f"Connected to GPU Server at {GPU_SERVER_IP}")

    cameras = DualRealSense()
    robot = DiffusionClient()

    
    # brain = AsyncInference(cameras, robot)

    # ensembler = TemporalEnsembler(horizon=15, action_dim=4, min_weight=0.01)    
    # Buffer stores dicts: {'c1': img, 'c2': img, 's': state}
    obs_buffer = deque(maxlen=2)

    try:
        while True:
            loop_start = time.time()
            
            # 1. Get Data (Instant, Non-Blocking)
            # The camera class now handles the deque/history internally
            # b_c1, b_c2 = cameras.get_observation_batch()
            # if b_c1 is None:
            #     time.sleep(0.01)
            #     continue 
            # b_s = robot.get_state_batch()

            # # # 1. Fill Buffer
            obs_buffer.clear()
            while len(obs_buffer) < 2:
                i1, i2 = cameras.get_frames()
                s = robot.get_state()
                obs_buffer.append({'c1': i1, 'c2': i2, 's': s})
            b_c1 = np.stack([x['c1'] for x in obs_buffer])
            b_c2 = np.stack([x['c2'] for x in obs_buffer])
            b_s = np.stack([x['s'] for x in obs_buffer])
            # 2. Stack Data
            # Shape: (2, 3, 240, 320)
            # print("got frames")
            # 3. Send to GPU (Dual Cam Payload)
            socket.send_pyobj({
                'cam1': b_c1.tobytes(),
                'cam2': b_c2.tobytes(),
                'state': b_s.tobytes(),
                'shape': (IMG_H, IMG_W)
            })

            # 4. Receive & Execute
            response = socket.recv_pyobj()
            actions = response['action'] # (16, 4)

            # print(f"Received {len(actions)} steps. Executing chunk...")
            # robot.execute(smooth_action)

            # elapsed = time.time() - loop_start
            # sleep_time = max(0, (1.0 / CONTROL_HZ) - elapsed)
            # time.sleep(sleep_time)


            # # 5. Sleep
            # elapsed = time.time() - start_t
            # time.sleep(max(0, (1.0/CONTROL_HZ) - elapsed))
            # --- C. EXECUTION LOOP (Action Chunking) ---
            # Execute all 8 steps blindly (Open-Loop execution)
            for i, action in enumerate(actions):
                step_start = time.time()
                
                # 1. Send Command
                robot.execute(action)
                
                # 2. Sleep to maintain 10Hz Control Rate
                elapsed = time.time() - step_start
                sleep_time = max(0, (1.0 / CONTROL_HZ) - elapsed)
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Stopping...")
        robot.robot.terminate_current_policy()

if __name__ == "__main__":
    main()