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

        kal_log = open("logs/kalman_log.csv", "w")
        sport_log = open("logs/sport_log.csv", "w")

        self.kf = KalmanFilter(dim_x=2, dim_z=1)

        self.kf.x = np.array([0., 0.,])  
        
        self.kf.F = np.array([[1., self.dt],
                              [0., 1]])
        
        self.kf.H = np.array([[0., 1.]])
        self.kf.P *= 100.  
        self.kf.R = 5
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.dt, var=1)
        while True:
            start_time = time.time()
            
            ddx, ddy, ddz = self.low_state.imu_state.accelerometer
            roll, pitch, yaw = self.low_state.imu_state.rpy
            gyrox, gyroy, gyroz = self.low_state.imu_state.gyroscope
            w, x, y, z = self.low_state.imu_state.quaternion

            z = np.array([ddx + 0.37])

            self.kf.predict()
            self.kf.update(z)
            
            kal_log.write(f"{self.kf.x[0]}, {self.kf.x[1]}\n")
            sport_log.write(f"{self.high_state.velocity[0]}, {self.high_state.velocity[1]}, {self.high_state.velocity[2]}\n")
            print(f"\r yaw: {yaw:.2f}, ddx : {ddx:+5.2f}, ddy: {ddy:+5.2f}, kalman_vel: {self.kf.x[0]:.2f}, kalman_acc: {self.kf.x[1]:.2f}", end='')
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