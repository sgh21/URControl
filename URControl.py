import rtde_receive
import rtde_control 
import utils.robotiq_gripper as robotiq_gripper
import utils.utils as utils
import time
import copy
import numpy as np
from ur_ikfast import ur_kinematics

class UR_controller():
    def __init__(self,robot_ip,step = 0.001,angle_step = 0.05,max_vel = 0.05,max_acc = 0.05,rtd_frequency =500 ,blend = 0.001):
        self.blend = blend 
        self.vel = max_vel  # 0.1
        self.acc =max_acc  # 0.3
        self.rtde_frequency = rtd_frequency
        self.ip = robot_ip
        self.step = step
        self.angle_step = angle_step
        self.flag = 0
        self.system_offset = np.zeros(6)
        self.control_freq = 0
        
    def init_system_offset(self):
        pose_vector = self.rtde_r.getActualTCPPose()
        joint = self.rtde_r.getActualQ()
        joint_q = self.robot_ik(pose_vector)

        self.system_offset += joint - joint_q

    def connect(self):
        if self.flag == 0:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip)
            print("connected ok")
            self.rtde_c = rtde_control.RTDEControlInterface(self.ip, self.rtde_frequency, rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP,30004)
            self.init_system_offset()
            print("connected ok")
            self.gripper = robotiq_gripper.RobotiqGripper()
            self.gripper.connect(self.ip, 63352)
            self.gripper.activate()
            self.flag = 1
        else:
            self.rtde_r.reconnect()
            self.rtde_c.reconnect()
            self.gripper.connect(self.ip, 63352)
            self.gripper.activate()

    def cliked_movex0(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[0] = tcp[0] + self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc,asynchronous=True)

    def cliked_movex1(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[0] = tcp[0] - self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_movey0(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[1] = tcp[1] + self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_movey1(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[1] = tcp[1] - self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_movez0(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[2] = tcp[2] + self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_movez1(self):
        tcp = self.rtde_r.getActualTCPPose()
        tcp[2] = tcp[2] - self.step
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRx0(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_x(self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRx1(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_x(-self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRy0(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_y(self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRy1(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_y(-self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRz0(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_z(self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)
    
    def cliked_moveRz1(self):
        tcp = self.rtde_r.getActualTCPPose()
        init_matrix = utils.vector2matrix(tcp)
        rotation_matrix = utils.rotate_z(-self.angle_step)
        temp_matrix = rotation_matrix @ init_matrix[:3, :3] 
        init_matrix[:3, :3] = temp_matrix
        tcp = utils.matrix2vector(init_matrix)
        self.rtde_c.moveL(tcp, self.vel,self.acc)

    def robot_ik(self,pose_vector):
        q_guess = self.rtde_r.getActualQ()
        # 获取真实坐标系下的目标位姿
        eef_tmat = utils.vector2matrix(pose_vector)
        # print('eef_tmat =\n', eef_tmat)
        
        tool_pose = [0,0,-0.157,0,0,0]
        T_tool = utils.vector2matrix(tool_pose)
        eef_tmat_ik = np.matmul(eef_tmat,T_tool)
        # ik坐标系下的目标位姿
        ik_offset = [0,0,0,0,0,np.pi]
        T_real_ik = utils.vector2matrix(ik_offset)
        eef_tmat_ik = np.matmul(np.linalg.inv(T_real_ik), eef_tmat_ik)
        # eef_tmat_ik = np.matmul(np.linalg.inv(T_real_ik), eef_tmat)  # 计算目标位姿在IK坐标系下的转换矩阵
        

        # 调用ur_kinematics库计算逆运动学
        ur_arm = ur_kinematics.URKinematics('ur5e')

        joint = ur_arm.inverse(eef_tmat_ik[:3], False,q_guess=q_guess) +self.system_offset # 计算逆运动学解
        # 返回解，如果没有解则返回joint_alternate
        # with np.printoptions(precision=3, suppress=True):
        #     print(np.array(q_guess)*180/np.pi-joint*180/np.pi)
        if joint is None:
            return q_guess
        return joint
    
    def safty_check(self,joint):
        joint_q = self.rtde_r.getActualQ()
        if np.max(np.abs(joint - joint_q)) > 0.1:
            return False
        return True
        
    def movel_servoj(self,pose,vel = 0.5,acc = 0.5,dt = 1.0/500,lookahead_time = 0.1,gain = 300):
        '''
        move the robot to the target pose using servoj
        The target pose is in the base frame and the target pose should be in the form of [x,y,z,rx,ry,rz]
        The target pose should be close to the current pose or the robot will not move
        '''
        target_pose = pose
        joint_q = self.robot_ik(target_pose)
        if not self.safty_check(joint_q):
            print("Warning:The target pose is too far from the current pose!")
            return
        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoJ(joint_q, vel, acc, dt, lookahead_time, gain)
        self.rtde_c.waitPeriod(t_start)

    def get_control_freq(self):
        joint_q = self.rtde_r.getActualQ()
        n_num = 1000
        for i in range(n_num):
            t1 = time.time()
            self.rtde_c.servoJ(joint_q,self.vel,self.acc,self.rtde_frequency,0.1,300)
            # joint_q[0] += 0.00001
            t2 = time.time()
        self.control_freq = (int)(n_num/(t2-t1))
        print(self.control_freq)

    def movel(self,target_pose,angle_step,threshold = 0.001):
        '''
        move the robot to the target pose 
        The target pose is in the base frame and the target pose should be in the form of [x,y,z,rx,ry,rz]
        The target pose have not to be close to the current pose
        '''
        # self.get_control_freq()
        start_joint = self.rtde_r.getActualQ()
        target_joint = self.robot_ik(target_pose)
        if max(np.abs(target_joint - start_joint)) > np.pi/2:
            raise ValueError("The target pose is too far from the current pose!")
        
        angle_diff = target_joint - start_joint
        angle_distance  = np.abs(angle_diff)
        current_joint = copy.deepcopy(start_joint)
        
        while np.max(np.abs(current_joint-target_joint)) > threshold:
            for i in range(len(current_joint)):    
                if abs(current_joint[i] - target_joint[i]) > angle_step:
                    current_joint[i] += angle_step*angle_diff[i]/angle_distance[i]
                elif abs(current_joint[i] - target_joint[i]) > threshold:
                    current_joint[i] = target_joint[i]
            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(current_joint,self.vel,self.acc,1/self.rtde_frequency,0.1,300)
            self.rtde_c.waitPeriod(t_start)
        
           

    def gripper_open(self):
        self.gripper.move_and_wait_for_pos(0, 150, 255)

    def gripper_close(self):
        self.gripper.move_and_wait_for_pos(255, 150, 255)

    def disconnect(self):
        self.rtde_r.disconnect()
        self.rtde_c.disconnect()
        self.gripper.disconnect()

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

