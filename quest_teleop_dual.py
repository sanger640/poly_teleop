import asyncio
import websockets
import json
import torch
import numpy as np
from polymetis import RobotInterface, GripperInterface
from scipy.spatial.transform import Rotation as R
import ssl
import time
import grpc
import pyrealsense2 as rs
import cv2
import os
import threading
import queue
import shutil

MAX_POSITION_STEP = 0.02   # smaller step for safety
WORKSPACE_RADIUS = 0.25    # slightly smaller workspace
WORKSPACE_MIN_Z = 0.08
ROBOT_BASE = np.array([0.0, 0.0, 0.0])

# --- CONFIGURATION ---
# REPLACE THESE WITH YOUR ACTUAL CAMERA SERIAL NUMBERS
CAMERA_1_SERIAL = "215222078938" 
CAMERA_2_SERIAL = "819612070440"
SSD_LOC="/mnt/diffusion_policy/tasks/pick_place/dual_wrist/"
# ---------------------

class MultiCameraRecorder:
    def __init__(self, i=0):
        self.i = i
        self.save_folder = SSD_LOC+"episodes/" + str(self.i) + "/rgb_frames/" 
        os.makedirs(self.save_folder, exist_ok=True)
        self.running = False
        
        # Pipelines for two cameras
        self.pipeline1 = rs.pipeline()
        self.config1 = rs.config()
        self.config1.enable_device(CAMERA_1_SERIAL)
        self.config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline2 = rs.pipeline()
        self.config2 = rs.config()
        self.config2.enable_device(CAMERA_2_SERIAL)
        self.config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.frame_queue = queue.Queue(maxsize=2000)

    def start(self):
        if not self.running:
            try:
                self.pipeline1.start(self.config1)
                print(f"Camera 1 ({CAMERA_1_SERIAL}) started.")
                self.pipeline2.start(self.config2)
                print(f"Camera 2 ({CAMERA_2_SERIAL}) started.")
            except RuntimeError as e:
                print(f"Error starting cameras: {e}")
                return

            self.running = True
            
            # Thread for capturing frames
            self.capture_thread = threading.Thread(target=self._capture_worker, daemon=True)
            self.capture_thread.start()
            
            # Thread for saving to disk (IO bound)
            self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
            self.save_thread.start()

    def _capture_worker(self):
        while self.running:
            # Wait for frames from both cameras
            # Note: This might reduce FPS to the slower camera
            frames1 = self.pipeline1.wait_for_frames()
            frames2 = self.pipeline2.wait_for_frames()
            
            color_frame1 = frames1.get_color_frame()
            color_frame2 = frames2.get_color_frame()
            
            if not color_frame1 or not color_frame2:
                continue
                
            img1 = np.asanyarray(color_frame1.get_data())
            img2 = np.asanyarray(color_frame2.get_data())
            timestamp = time.time()
            
            try:
                # Put both images in the queue
                self.frame_queue.put_nowait((timestamp, img1, img2))
            except queue.Full:
                pass # Drop frame if disk is too slow

    def _save_worker(self):
        while self.running or not self.frame_queue.empty():
            try:
                timestamp, img1, img2 = self.frame_queue.get(timeout=0.1)
                
                # Save Camera 1
                fname1 = os.path.join(self.save_folder, f"cam1_{int(timestamp * 1000)}.png")
                cv2.imwrite(fname1, img1)
                
                # Save Camera 2
                fname2 = os.path.join(self.save_folder, f"cam2_{int(timestamp * 1000)}.png")
                cv2.imwrite(fname2, img2)
                
                self.frame_queue.task_done()
            except queue.Empty:
                pass

    def stop(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        if hasattr(self, 'save_thread'):
            self.save_thread.join()
        
        try:
            self.pipeline1.stop()
            self.pipeline2.stop()
        except:
            pass
        print("Cameras stopped.")

class AdvancedQuestRobotTeleop:
    """Advanced teleoperation with position AND orientation control"""

    def __init__(self, enable_orientation=False):
        print("Connecting to robot...")
        self.robot = RobotInterface(ip_address="129.97.71.27")
        self.gripper = GripperInterface(ip_address="129.97.71.27")
        # Control flags
        self.enable_orientation = enable_orientation

        # Start impedance controller (softer gains)
        self.Kx = torch.Tensor([750, 750, 750, 15, 15, 15])
        self.Kxd = torch.Tensor([37, 37, 37, 2, 2, 2])
        self.robot.start_cartesian_impedance(Kx=self.Kx, Kxd=self.Kxd)
        print("Robot impedance controller started!")

        # Get initial robot pose
        self.initial_robot_pos, self.initial_robot_quat = self.robot.get_ee_pose()
        print(f"Initial robot EE position: {self.initial_robot_pos.numpy()}")
        print(f"Orientation control: {'ENABLED' if enable_orientation else 'DISABLED'}")

        # Calibration variables
        self.initial_controller_pos = None
        self.initial_controller_rot = None
        self.initial_robot_rot = None
        self.calibrated = False
        self.prev_target_pos = None
        self.prev_target_quat = None

        # Control parameters (slow & gentle)
        self.position_scale = np.array([1.2, 1.2, 2.2])
        self.rotation_scale = 0.3  

        # For control-rate throttling (~10 Hz)
        self.last_command_time = 0.0
        self.command_period = 0.1

        # For velocity limiting / dt
        self.prev_time = time.time()
        self.max_velocity = 0.2  # 5 cm/s EE velocity

        # For input change detection
        self.prev_controller_pos = None
        self.prev_controller_quat = None

        # Deadband parameters
        self.DEADBAND_POS = 0.01  
        self.DEADBAND_EE = 0.005  
        self.DEADBAND_ORI_RAD = np.deg2rad(3) 

        # Recording
        self.recording = False
        self.trajectory_data = []
        self.record_start_time = None
        self._grip_pressed  = False
        # Gripper state
        self.last_gripper_closed = False

        # Initialize Multi-Camera Recorder
        self.i = 1
        self.recorder = MultiCameraRecorder(self.i)

        self.track = False
        self.pos_delta = 0

    def transform_controller_to_robot(self, pos):
        # Map: Robot X = -Controller Z, Robot Y = -Controller X, Robot Z = Controller Y
        return np.array([-pos[0], pos[2], pos[1]])

    def transform_controller_quat(self, q):
        # Input: [x, y, z, w] (controller)
        # Output: [z, x, y, w] (robot) with axis swap and sign flips
        return np.array([-q[0], q[2], q[1], q[3]])

    def calibrate(self, controller_pos, controller_quat):
        """Calibrate coordinate systems"""
        self.initial_controller_pos = self.transform_controller_to_robot(controller_pos)
        self.initial_controller_rot = R.from_quat(self.transform_controller_quat(controller_quat))
        self.initial_robot_rot = R.from_quat(self.initial_robot_quat.numpy())
        self.prev_controller_pos = controller_pos.copy()
        self.prev_controller_quat = controller_quat.copy()
        self.calibrated = True
        self.prev_target_pos = self.initial_robot_pos.numpy()
        self.prev_target_quat = self.initial_robot_quat.numpy()

        print(f"\n{'='*50}")
        print("CALIBRATED!")
        print(f"Controller position: {controller_pos}")
        print(f"Robot EE position: {self.initial_robot_pos.numpy()}")
        print(f"{'='*50}\n")
        print("Ready to teleoperate! Move your controller.")

    async def handle_controller_data(self, websocket):
        """Process incoming controller data"""
        print("\nQuest controller connected!")
        print("Waiting for first controller input to calibrate...")

        async for message in websocket:
            try:
                # --- Throttle control rate to ~10 Hz ---
                now = time.time()
                if now - self.last_command_time < self.command_period:
                    continue
                self.last_command_time = now

                data = json.loads(message)

                # Extract controller data
                controller_pos = np.array(data['position'])
                controller_quat = np.array(data['orientation'])
                trigger_value = data.get('trigger', 0.0)
                grip_button = data.get('grip', 0.0)
                a_butt = data.get('button_a', 0.0)
                b_butt = data.get('button_b', 0.0)

                if a_butt > 0.5 and self.track == False:
                    self.track = True
                    # calibrate
                    controller_pos_robot = self.transform_controller_to_robot(controller_pos)
                    self.initial_controller_pos = controller_pos_robot - (self.pos_delta/self.position_scale)
                if b_butt > 0.5 and self.track == True:
                    self.track = False

                if self.track == True:

                    # Calibrate on first message
                    if not self.calibrated:
                        self.calibrate(controller_pos, controller_quat)
                        continue

                    # === POSITION DEADBAND ON CONTROLLER INPUT ===
                    controller_pos_robot = self.transform_controller_to_robot(controller_pos)
                    delta_ctrl = np.linalg.norm(controller_pos_robot - self.prev_controller_pos) if self.prev_controller_pos is not None else np.inf
                    if delta_ctrl < self.DEADBAND_POS:
                        # Use last controller pos in robot frame
                        controller_pos_robot = self.prev_controller_pos.copy()

                    # Calculate position delta and EE target position
                    self.pos_delta = (controller_pos_robot - self.initial_controller_pos) * self.position_scale
                    target_pos = self.initial_robot_pos.numpy() + self.pos_delta

                    # === DEAD BAND ON EE TARGET POSITION ===
                    if self.prev_target_pos is not None:
                        delta_ee = np.linalg.norm(target_pos - self.prev_target_pos)
                        if delta_ee < self.DEADBAND_EE:
                            target_pos = self.prev_target_pos.copy()

                    # === VELOCITY LIMITING ===
                    dt = now - self.prev_time
                    if dt > 0 and self.prev_target_pos is not None:
                        dist = np.linalg.norm(target_pos - self.prev_target_pos)
                        vel = dist / dt
                        if vel > self.max_velocity and dist > 1e-6:
                            direction = (target_pos - self.prev_target_pos) / dist
                            target_pos = self.prev_target_pos + direction * self.max_velocity * dt
                            # print(f"[VELOCITY LIMIT] Clamped EE speed to {self.max_velocity:.3f} m/s")
                    self.prev_time = now

                    self.prev_target_pos = target_pos.copy()
                    target_pos_tensor = torch.Tensor(target_pos)

                    # === ORIENTATION CONTROL WITH DEADBAND ===
                    if self.enable_orientation:
                        current_controller_rot = R.from_quat(self.transform_controller_quat(controller_quat))
                        delta_rot = current_controller_rot * self.initial_controller_rot.inv()
                        
                        angle = delta_rot.magnitude()  # rotation angle in radians
                        if angle < self.DEADBAND_ORI_RAD and self.prev_target_quat is not None:
                            target_quat = self.prev_target_quat.copy()
                        else:
                            target_rot = delta_rot * self.initial_robot_rot
                            target_quat = target_rot.as_quat()
                        
                        self.prev_target_quat = target_quat.copy()
                        target_quat_tensor = torch.Tensor(target_quat)
                    else:
                        target_quat_tensor = self.initial_robot_quat

                    self.prev_controller_pos = controller_pos.copy()
                    self.prev_controller_quat = controller_quat.copy()

                    # Update robot pose with error recovery
                    try:
                        self.robot.update_desired_ee_pose(
                            position=target_pos_tensor,
                            orientation=target_quat_tensor
                        )
                    except grpc.RpcError as e:
                        msg = str(e)
                        if "no controller running" in msg or "power_limit_violation" in msg or "Safety limits exceeded" in msg:
                            print(f"[ERROR] {msg}. Restarting Cartesian impedance controller...")
                            self.robot.start_cartesian_impedance(Kx=self.Kx, Kxd=self.Kxd)
                            self.robot.update_desired_ee_pose(
                                position=target_pos_tensor,
                                orientation=target_quat_tensor
                            )
                        else:
                            raise e

                    # Gripper control
                    gripper_closed = trigger_value > 0.5
                    if gripper_closed != self.last_gripper_closed:
                        if gripper_closed:
                            print("Gripper Closed")
                            self.gripper.grasp(speed=0.05, force=0.1)
                        else:
                            print("Gripper Open")
                            self.gripper.stop()
                            self.gripper.goto(width=0.25, speed=0.05, force=0.1)
                        self.last_gripper_closed = gripper_closed

                    # Grip button logic to start/stop recording
                    if grip_button > 0.5 and not self._grip_pressed:
                        self._grip_pressed = True
                        self.recording = True
                        self.trajectory_data = []
                        self.record_start_time = time.time()
                        self.recorder.start()
                        print(f"Grip pressed. Starting recording. Episode {self.i}/{self.recorder.i}")
                    elif grip_button <= 0.5 and self._grip_pressed:
                        self._grip_pressed = False
                        self.recording = False
                        self.save_trajectory()
                        self.recorder.stop()
                        self.recorder.i += 1
                        self.recorder.save_folder = SSD_LOC+"episodes/" + str(self.i) + "/rgb_frames/"
                        os.makedirs(self.recorder.save_folder, exist_ok=True)
                        print(f"\n⏹ RECORDING STOPPED ({len(self.trajectory_data)} waypoints)")

                    if self.recording:
                        # Record the command we just sent (target pose)
                        if self.enable_orientation:
                            self.record_waypoint(target_pos_tensor, target_quat_tensor, gripper_closed)
                        else:
                            self.record_waypoint(target_pos_tensor, self.initial_robot_quat, gripper_closed)

            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except Exception as e:
                print(f"Error processing data: {e}")
                import traceback
                traceback.print_exc()

    def record_waypoint(self, position, orientation, gripper):
        timestamp = time.time()
        self.trajectory_data.append({
            'timestamp': timestamp,
            'position': position.numpy().tolist(),
            'orientation': orientation.numpy().tolist(),
            'gripper': gripper
        })

    def save_trajectory(self):
        if not self.trajectory_data:
            print("No data to save!")
            return

        import time as _time
        filename = f"{SSD_LOC}episodes/{self.i}/trajectory_{int(_time.time())}.json"

        trajectory = {
            'metadata': {
                'num_waypoints': len(self.trajectory_data),
                'duration': self.trajectory_data[-1]['timestamp'],
                'orientation_enabled': self.enable_orientation
            },
            'waypoints': self.trajectory_data
        }

        with open(filename, 'w') as f:
            json.dump(trajectory, f, indent=2)

        print(f"✓ Saved trajectory to: {filename}")
        self.i +=1

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(certfile='cert.pem', keyfile='key.pem')

async def main():
    teleop = AdvancedQuestRobotTeleop(enable_orientation=False)
    async with websockets.serve(teleop.handle_controller_data, '0.0.0.0', 8765, ssl=ssl_context):
        print('WSS server running on port 8765')
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())