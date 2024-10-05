## Environments
+ Supported system:Ubuntu and other Linux distributions.(Windows is supported if you use other inverse kinematics solvers.)

## Dependency
+ [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html):Used for real-time data exchange with UR and taking over the control of UR.
+ [ur_ikfast](https://github.com/cambel/ur_ikfast/):Used for fast inverse kinematics solutions for UR series robots.
### Optional
+ [pyspacemouse](https://github.com/JakubAndrysek/PySpaceMouse/tree/master)
:pyspacemouse is needed if you want to control robot using 3Dconnecion SpaceMouse

## Installion
### Install [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html)
```shell
# install boost on Ubuntu
sudo apt-get install libboost-all-dev
# clone the remote repository and build
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
mkdir build
cd build
cmake ..
make
sudo make install
# if you want to use it by Python make sure you have installed pybind11 
# install Python package
pip install --user ur_rtde

```
### Install [ur_ikfast](https://github.com/cambel/ur_ikfast/)
1. install dependencies
```shell
sudo apt-get install libblas-dev liblapack-dev
pip install --user numpy Cython
```

3. install using pip </br>

```shell
git clone https://github.com/cambel/ur_ikfast.git
cd ur_ikfast
pip install -e .
```
### Install [pyspacemouse](https://github.com/JakubAndrysek/PySpaceMouse/tree/master)(Optional)

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install [pyspacemouse](https://pypi.org/project/pyspacemouse/). If you are using a Mac with an ARM processor, you'll need a patched version of `easyhid`.

```shell
# Install package
pip install pyspacemouse

# Only needed for ARM MacOs
pip install git+https://github.com/bglopez/python-easyhid.git
```

#### Dependencies (required)

The library uses `hidapi` as low-level interface to the device and `easyhid` as a Python abstraction for easier use.

- ##### [hidapi](https://github.com/libusb/hidapi) is `C` library for direct communication with HID devices
    - #### Linux
        - [libhidapi-dev]() to access HID data
        - `sudo apt-get install libhidapi-dev` (Debian/Ubuntu)
        - Compile and install [hidapi](https://github.com/libusb/hidapi/#build-from-source).  (other Linux
          distributions)

        - add rules for permissions
            ```bash
            sudo echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' > /etc/udev/rules.d/99-hidraw-permissions.rules
            sudo usermod -aG plugdev $USER
            newgrp plugdev
            ```
            <details>
            <summary>Aleternative option - with tee (RPi)</summary>
            <pre>
            echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-hidraw-permissions.rules
            sudo usermod -aG plugdev $USER
            newgrp plugdev
            </pre>
            </details>

## Example
+ Move use SpaceMouse
```python
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
```
+ Run 
```shell
cd ./demo
python demo_ur5e_movel_servoJ.py
```