import time
import sys
import onnxruntime as ort
import numpy as np 
import traceback
import os
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from math import sqrt, pi
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

import dashboard


class Custom:
    def __init__(self):

        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None
        self.high_state = None 
        self.dt = 0.02 
     
        # thread handling
        self.lowCmdWriteThreadPtr = None

        self.crc = CRC()

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state : LowState_ = msg
    
    def HighStateMessageHandler(self, msg: SportModeState_):
        self.high_state : SportModeState_ = msg
        # print("velocity: ", self.high_state.velocity)

    def Init(self):
        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.highstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.highstate_subscriber.Init(self.HighStateMessageHandler, 10)


        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()


        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        
        status, result = self.msc.CheckMode()

        acc_log = open("logs/acc_log.csv", "w")
        acc_log.write("yaw, ddx, ddy, ddz\n")
        
        yaw_log = open("logs/yaw_log.csv", "a")

        totals = [0, 0, 0]
        points = 0

        while True:

            start_time = time.time()
            ddx, ddy, ddz = self.low_state.imu_state.accelerometer
            roll, pitch, yaw = self.low_state.imu_state.rpy
            gyrox, gyroy, gyroz = self.low_state.imu_state.gyroscope
            w, x, y, z = self.low_state.imu_state.quaternion

            if points == 0:
                start_yaw = yaw

            points += 1
            totals[0] += ddx
            totals[1] += ddy
            totals[2] += ddz

            if abs(yaw - start_yaw) > .01:
                print("\ryaw offset detected: ", yaw - start_yaw)
                points = 0
                totals = [0, 0, 0]
                continue

            print(f"\rAvg: {totals[0]/points:.4f}, {totals[1]/points:.4f}, {totals[2]/points:.4f} | Points: {points} | Yaw: {yaw:.4f}", end='', flush=True)
            if points == 30: 
                print(f"\rLogging Avg: {totals[0]/points:.4f}, {totals[1]/points:.4f}, {totals[2]/points:.4f} | Yaw: {yaw:.4f} ")
                yaw_log.write(f"{yaw}, {totals[0]/points}, {totals[1]/points} \n")
                points = 0
                totals = [0, 0, 0]

            elapsed = time.time() - start_time
            if self.dt - elapsed > 0:
                time.sleep(self.dt - elapsed)
            

    def Start(self):
        pass

if __name__ == '__main__':

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
    print("Finished Example. Stopping...")
    # while True:        
    #     time.sleep(1)