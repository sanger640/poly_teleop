import numpy as np
import cv2
import pyrealsense2 as rs

class DualRealSense:
    def __init__(self, cam1_serial, cam2_serial, H, W, hz):
        # Cam 1
        self.cam1_serial = cam1_serial
        self.cam2_serial = cam2_serial
        self.H = H 
        self.W = W
        self.hz = hz
        self.p1 = rs.pipeline()
        c1 = rs.config()
        c1.enable_device(self.cam1_serial)
        c1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.hz)
        
        # Cam 2
        self.p2 = rs.pipeline()
        c2 = rs.config()
        c2.enable_device(self.cam2_serial)
        c2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.hz)
        
        self.p1.start(c1)
        self.p2.start(c2)
        print("Dual Cameras Started")

    def get_frames(self):
        # sync
        f1 = self.p1.wait_for_frames()
        f2 = self.p2.wait_for_frames()
        
        i1 = np.asanyarray(f1.get_color_frame().get_data())
        i2 = np.asanyarray(f2.get_color_frame().get_data())
        
        # resize and rgb
        i1 = cv2.resize(i1, (self.W, self.H))
        i1 = cv2.cvtColor(i1, cv2.COLOR_BGR2RGB)
        
        i2 = cv2.resize(i2, (self.W, self.H))
        i2 = cv2.cvtColor(i2, cv2.COLOR_BGR2RGB)
        
        # (C, H, W)
        i1 = np.transpose(i1, (2, 0, 1))
        i2 = np.transpose(i2, (2, 0, 1))
        
        return i1, i2