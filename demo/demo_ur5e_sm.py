import sys
sys.path.append('..')
import utils.utils as utils
import time
import numpy as np
from utils.SpacemouseExpert import SpaceMouseExpert
from URControl import UR_controller


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
    i = 0
    while True:
        with np.printoptions(precision=3, suppress=True):
            action, buttons = spacemouse.get_action()
            if buttons[1]:
                    print("This program is terminaled.")
                    break
            if buttons[0]:
                gripper_flag = not gripper_flag
                if gripper_flag:
                    ur_controller.gripper_open()
                else:
                    ur_controller.gripper_close()
            if np.sum(np.abs(action))<0.000001:
                pose_vector = ur_controller.rtde_r.getActualTCPPose()
            else:
                action[:3] = action[:3] / scale_move
                action[3:] = action[3:] / scale_rotation
                init_matrix = utils.vector2matrix(pose_vector)
                rotation_matrix_x = utils.rotate_x(action[3])
                rotation_matrix_y = utils.rotate_y(action[4])
                rotation_matrix_z = utils.rotate_z(action[5])
                temp_matrix = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x @ init_matrix[:3, :3] 
                init_matrix[:3, :3] = temp_matrix
                pose_vector = utils.matrix2vector(init_matrix)
                pose_vector[:3] = pose_vector[:3] + action[:3]
                ur_controller.movel_servoj(pose_vector)
            i += 1
    time2=time.time()
    print((int)(i/(time2-time1)))

    ur_controller.disconnect()







# 主循环

