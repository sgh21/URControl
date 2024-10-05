#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time         : 2024/10/3 下午10:24
# @Author       : Wang Song
# @File         : spacemouse_test.py
# @Software     : PyCharm
# @Description  :
""" Test the spacemouse output. """
import time
import numpy as np
from utils.SpacemouseExpert import SpaceMouseExpert


def test_spacemouse():
    """Test the SpaceMouseExpert class.

    This interactive test prints the action and buttons of the spacemouse at a rate of 10Hz.
    The user is expected to move the spacemouse and press its buttons while the test is running.
    It keeps running until the user stops it.

    """
    spacemouse = SpaceMouseExpert()
    with np.printoptions(precision=3, suppress=True):
        while True:
            action, buttons = spacemouse.get_action()
            print(f"Spacemouse action: {action}, buttons: {buttons}")
            time.sleep(0.1)

def sm_frequency():
    """Test the SpaceMouseExpert class.

    This interactive test prints the action and buttons of the spacemouse at a rate of 10Hz.
    The user is expected to move the spacemouse and press its buttons while the test is running.
    It keeps running until the user stops it.

    """
    spacemouse = SpaceMouseExpert()
    with np.printoptions(precision=3, suppress=True):
        time1 = time.time()
        for i in range(500):
            action, buttons = spacemouse.get_action()
        time2 = time.time()
        print((int)(500/(time2-time1)))
            # print(f"Spacemouse action: {action}, buttons: {buttons}")
            # time.sleep(0.1)

def main():
    """Call spacemouse test."""
    # test_spacemouse()
    sm_frequency()


if __name__ == "__main__":
    main()