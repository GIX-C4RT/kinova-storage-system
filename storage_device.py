from time import time
from contextlib import contextmanager

import cv2
import numpy as np
import imutils

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from arm_control import cartesian_action_movement_absolute, move_to_home_position, set_gripper_position, twist_command, cartesian_action_movement_relative

import utilities
from pid import PID

ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL

aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters_create()

class StorageDevice:
    def __init__(self, ip_address="192.168.1.10", username="admin", password="admin"):
        '''Return an instance of the Arm class. Must be used in a with statement.'''
        self.ip = ip_address
        self.username = username
        self.password = password
    
    
    def __enter__(self):
        '''Connect to arm and open video stream.'''
        self.device_connection = utilities.DeviceConnection.createTcpConnection(self)
        self.router = self.device_connection.__enter__()
        # Create required services
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        # open video stream
        self.cap = cv2.VideoCapture("rtsp://" + self.ip + "/color")

    
    def __exit__(self, exc_type, exc_value, traceback):
        '''Disconnect from arm and close video stream.'''
        self.device_connection.__exit__(None, None, None)
        self.cap.release()

    def pick(self, id, aruco_dict):
        pass


    def place(self, id, aruco_dict):
        pass




if __name__ == "__main__":
    with StorageDevice() as device:
        print("With device!")