import sys
sys.path.append('..')
from URControl import UR_controller
from utils.SpacemouseExpert import SpaceMouseExpert
import time
import numpy as np
if __name__ =="__main__":
# global ur_control
# 设置一个全局变量来控制循环
    robot_ip = "192.168.1.10"
    ur_controller = UR_controller(robot_ip)
    spacemouse = SpaceMouseExpert()
    ur_controller.connect()
    gripper_flag = True
    scale_move = 100000/20
    scale_rotation = 10000/10
    pose_vector = ur_controller.rtde_r.getActualTCPPose()
    time1=time.time()

    while True:
        with np.printoptions(precision=3, suppress=True):
            action, buttons = spacemouse.get_action()
            if buttons[1]:
                    print("This program is terminaled.")
                    break
            if buttons[0]:
                pose_vector = ur_controller.rtde_r.getActualTCPPose()
                target_pose_step = np.array([0.2,0,0,0,0,0])
                target_pose = pose_vector + target_pose_step
                ur_controller.movel(target_pose,angle_step=0.001)
            
    time2=time.time()
    # print((int)(i/(time2-time1)))

    ur_controller.disconnect()