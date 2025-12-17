import numpy as np
from polymetis import RobotInterface, GripperInterface

def main():
    # use ethernet port of the host
    robot = RobotInterface(ip_address="129.97.71.27")
    gripper = GripperInterface(ip_address="129.97.71.27")
    target_position = np.array([0.5579355,   0.19265589,  0.3486197])
    target_orientation = np.array([0.9332133531570435,
        -0.35531800985336304,
        0.04004822298884392,
        0.035469427704811096])  # Quaternion (x,y,z,w)

    # Set target EE pose
    robot.set_ee_pose(target_position, target_orientation)

    # Run control loop until target reached or timeout
    # for _ in range(1000):
    #     robot.step()  # send control commands
    # robot.disconnect()

    gripper.stop()
    gripper.goto(width=0.25, speed=0.05, force=0.1)
if __name__ == "__main__":
    main()