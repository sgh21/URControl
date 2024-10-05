import sys
sys.path.append('..')
import time
import numpy as np
from URControl import UR_controller

if __name__ =="__main__":
# global ur_control
    robot_ip = "192.168.1.10"
    ur_controller = UR_controller(robot_ip)
    ur_controller.connect()
    # pose_vector = ur_controller.rtde_r.getActualTCPPose()
    # ur_controller.ur5e_ik(pose_vector)
    target_pose_step = np.array([0,0,0.10,0,0,0])
    n_num = 500
    t1 = time.time()
    pose_vector = ur_controller.rtde_r.getActualTCPPose()
    for i in range(n_num):
        step =target_pose_step/n_num
        pose_vector += step
        ur_controller.movel_servoj(pose_vector)
    t2 = time.time()
    print((int)(500/(t2-t1)))
    time.sleep(0.1)
    target_pose_step = np.array([0,0,-0.05,0,0,0])
    n_num = 500
    t1 = time.time()
    pose_vector = ur_controller.rtde_r.getActualTCPPose()
    for i in range(n_num):
        step =target_pose_step/n_num
        pose_vector += step
        ur_controller.movel_servoj(pose_vector)
    t2 = time.time()
    print((int)(500/(t2-t1)))
    time.sleep(0.1)
    target_pose_step = np.array([0,0.05,0,0,0,0])
    n_num = 500
    t1 = time.time()
    pose_vector = ur_controller.rtde_r.getActualTCPPose()
    for i in range(n_num):
        step =target_pose_step/n_num
        pose_vector += step
        ur_controller.movel_servoj(pose_vector)
    t2 = time.time()
    print((int)(500/(t2-t1)))
    time.sleep(0.1)
    target_pose_step = np.array([0,-0.10,0,0,0,0])
    n_num = 500
    t1 = time.time()
    pose_vector = ur_controller.rtde_r.getActualTCPPose()
    for i in range(n_num):
        step =target_pose_step/n_num
        pose_vector += step
        ur_controller.movel_servoj(pose_vector)
    t2 = time.time()
    print((int)(500/(t2-t1)))
    
    ur_controller.disconnect()

