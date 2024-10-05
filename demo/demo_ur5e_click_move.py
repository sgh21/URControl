import sys
sys.path.append('..')
from URControl import UR_controller

if __name__ =="__main__":
# global ur_control
    robot_ip = "192.168.1.10"
    ur_controller = UR_controller(robot_ip)
    ur_controller.connect()
    while True:
        order =  input("Enter the order you want")
        if order == 'x+':
            ur_controller.cliked_movex0()
        if order == 'x-':
            ur_controller.cliked_movex1()   
        if order == 'y+':
            ur_controller.cliked_movey0()
        if order == 'y-':
            ur_controller.cliked_movey1()
        if order == 'z+':
            ur_controller.cliked_movez0()
        if order == 'z-':
            ur_controller.cliked_movez1()  
        if order == 'rx+':
            ur_controller.cliked_moveRx0()
        if order == 'rx-':
            ur_controller.cliked_moveRx1()
        if order == 'ry+':
            ur_controller.cliked_moveRy0()
        if order == 'ry-':
            ur_controller.cliked_moveRy1()
        if order == 'rz+':
            ur_controller.cliked_moveRz0()
        if order == 'rz-':
            ur_controller.cliked_moveRz1()
        if order == 'o':
            ur_controller.gripper_open()
        if order == 'c':
            ur_controller.gripper_close()
        if order == '.':
            break
    ur_controller.disconnect()

