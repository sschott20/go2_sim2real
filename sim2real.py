import time
import sys
import onnxruntime as ort
import numpy as np 
import traceback
import os


from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

import dashboard
ACTION_SCALE = 0.25
SIM_DT = 0.005
SIM_DECIMATION = 4

class Custom:
    def __init__(self):
        # stand command parameters
        self.Kp = 60.0
        self.Kd = 5.0
        self.time_consume = 0
        self.rate_count = 0
        self.sin_count = 0
        self.motiontime = 0
        self.dt = SIM_DT * SIM_DECIMATION
        self.run_time = 0
        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  
        self.high_state = None  

        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self._targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]
        
        self._display_pose = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self._default_stance = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]
        
        self._offset = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]).reshape(1, 12)
        self._isaaclab_offset = np.array([[0.1000, -0.1000, 0.1000, -0.1000, 0.8000, 0.8000, 1.0000, 1.0000, -1.5000, -1.5000, -1.5000, -1.5000]], dtype=np.float32)
     
        # thread handling
        self.lowCmdWriteThreadPtr = None
        self.kf = None
        self.crc = CRC()

    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state : LowState_ = msg

   
    def InitKalmanFilter(self):
        
        self.kf = KalmanFilter(dim_x=3, dim_z=1)
        self.kf.x = np.array([0., 0, 0, 0])
        self.kf.F = np.array(   [[1., self.dt, 0.5 * self.dt**2],
                                [0., 1., self.dt],
                                [0., 0., 1.]])
        self.kf.H = np.array([[0., 1., 0.]])
        self.kf.P *= 1000.
        self.kf.R = .1
        self.kf.Q = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.1)

        

    def Init(self):

        self.InitLowCmd()
        self.LoadModel()
        self.InitKalmanFilter()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()

        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)


    def SetCommand(self, x = 0.0, y= 0.0, z=0.0):
        self.vel_cmd[0, 0] = x
        self.vel_cmd[0, 1] = y
        self.vel_cmd[0, 2] = z 

    def Start(self):
        # self.Stand(self._default_stance)
        # self.SetJointsSmooth(self._targetPos_1, 100)
        self.SetJointsSmooth(self._default_stance, 100)


        self.SetCommand(0.5, 0.0, 0.0)
        try:
            for i in range(300):
                
                self.run_time += 1
                start_time = time.time()

                self.GetObservations()
                self.GetActions()

                # self.robot_frame_raw_actions = self.raw_actions[np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])]

                # self.PrintObs()
                # self.PrintActions()
                stable = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]

                for i in range(12):
                    self.SetJointAction(i, stable[i])

                self.robot_frame_actions = self.actions[0][np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])]
                # print(self.robot_frame_actions[0])
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self.robot_frame_actions[i] 
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = 25
                    self.low_cmd.motor_cmd[i].kd = 0.5
                    self.low_cmd.motor_cmd[i].tau = 0

                # self.low_cmd.motor_cmd[1].q = 0.8
                # self.low_cmd.motor_cmd[1].dq = 0
                # self.low_cmd.motor_cmd[1].kp = 0
                # self.low_cmd.motor_cmd[1].kd = 0
                # self.low_cmd.motor_cmd[1].tau = 0.01

                # self.ordered_efforts = self.GetEfforts()

                # self.SetEffortCmd()

                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.lowcmd_publisher.Write(self.low_cmd)   

                sleep_time = self.dt - (time.time() - start_time)
                if sleep_time > 0:
                    # print("sleep: ", sleep_time)
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("KeyboardInterrupt: Stopping inference.")
        except Exception as e:
            print("An error occurred during inference:")
            print(traceback.format_exc())

        print("Settings joints to default stance.")
        self.SetJointsSmooth(self._default_stance, 100)
        print("Setting joints to sitting position.")
        self.SetJointsSmooth(self._targetPos_3, 100, start_pos=self._default_stance)
        time.sleep(.5)

        self.msc.SelectMode("normal")
        status, result = self.msc.CheckMode()
        self.sc.StandUp()
        time.sleep(1)
    
    def PrintObs(self):
        print("Observations:")
        print("Angular Velocity:", self.ang_vel)
        print("Projected Gravity:", self.proj_gravity)
        print("Velocity Command:", self.vel_cmd)
        print("Joint Positions (relative):", self.joint_pos_rel)
        print("Joint Velocities:", self.joint_vel)
        print("Last Actions:", self.last_actions)  
        print("===\n")
        
    def PrintActions(self):
        print("Actions:")
        print("FR: ", self.actions[0, 0:3])
        print("FL: ", self.actions[0, 3:6])
        print("RR: ", self.actions[0, 6:9])
        print("RL: ", self.actions[0, 9:12])
        print("raw actions: ")
        print("hip: ", self.raw_actions[0:4])
        print("thigh: ", self.raw_actions[4:8])
        print("calf: ", self.raw_actions[8:12])
    

    # Private methods
    def LoadModel(self):
        self.obs = np.zeros((1, 45), dtype=np.float32)  
        self.actions = np.zeros((1, 12), dtype=np.float32)  

        self.pos_history = []  

        # self.lin_vel = np.zeros((1, 3), dtype=np.float32) 
        self.ang_vel = np.zeros((1, 3), dtype=np.float32)
        self.proj_gravity = np.zeros((1, 3), dtype=np.float32)  
        self.vel_cmd = np.zeros((1, 3), dtype=np.float32)
        self.joint_pos_rel = np.zeros((1, 12), dtype=np.float32)
        self.joint_vel = np.zeros((1, 12), dtype=np.float32)
        self.last_actions = np.zeros((1, 12), dtype=np.float32)
        self.joint_pos_abs = np.zeros((1, 12), dtype=np.float32)
        self.raw_actions = np.zeros((1, 12), dtype=np.float32)



        self.ort_sess = ort.InferenceSession("models/aug25-low.onnx")


    def GetObservations(self):

        self.pos_history.append((self.low_state.imu_state.quaternion[:3], time.time()))
        # self.lin_vel[0, :] = self.GetVelocity()

        self.ang_vel[0, :] = self.low_state.imu_state.gyroscope
        self.proj_gravity[0, 2] = -1
        self.vel_cmd[0, :] = self.vel_cmd[0, :] 

        # Conversion from Real -> Isaac:[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
        # Conversion From Isaac -> Real:[1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10]
        # this shit needs to be here because the joints are indexed in issaaclab acording to above, but indexed in unitree sdk like this:
        # Isaacalab
        # 0FLhip, 1FRhip, 2BLhip, 3BRhip, 4FLthigh, 5FRthigh, 6BLthigh, 7BRthigh, 8FLclaf, 9FRcalf, 10BLclaf, 11BRcalf])
        # ([[ 0.1000, -0.1000,  0.1000, -0.1000,  0.8000,  0.8000,  1.0000,  1.0000, -1.5000, -1.5000, -1.5000, -1.5000]
        # self.offset = np.array([[0.1000, -0.1000, 0.1000, -0.1000, 0.8000, 0.8000, 1.0000, 1.0000, -1.5000, -1.5000, -1.5000, -1.5000]], dtype=np.float32)
        #  Real life
        #  FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2
        #  FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
        #  RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
        #  RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11

        
        self.joint_pos_abs = np.zeros((1, 12), dtype=np.float32)
        self.joint_pos_abs[0, :] = [self.low_state.motor_state[i].q for i in [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]]
        self.joint_pos_rel = self.joint_pos_abs - self._isaaclab_offset

        self.joint_vel[0, :] = [self.low_state.motor_state[i].dq for i in [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]]

        # self.last_actions[0, :] = self.last_actions[0, :]

        self.obs[0, 0:3] = self.ang_vel
        self.obs[0, 3:6] = self.proj_gravity
        self.obs[0, 6:9] = self.vel_cmd
        self.obs[0, 9:21] = self.joint_pos_rel
        self.obs[0, 21:33] = self.joint_vel
        self.obs[0, 33:45] = self.last_actions

        # self.obs[0, 0:3] = self.lin_vel
        # self.obs[0, 3:6] = self.ang_vel
        # self.obs[0, 6:9] = self.proj_gravity
        # self.obs[0, 9:12] = self.vel_cmd
        # self.obs[0, 12:24] = self.joint_pos_rel
        # self.obs[0, 24:36] = self.joint_vel
        # self.obs[0, 36:48] = self.last_actions

    def GetEfforts(self):
        stiffness = 25.0
        damping = 5
        effort_limit = 23.5
        saturation_effort = 23.5
        velocity_limit = 30.0

        error_pos = self.actions[0] - self.joint_pos_abs
        error_vel = np.zeros(12).reshape(1, 12) - self.joint_vel
        computed_effort = stiffness * error_pos + damping * error_vel # + control effort = 0

        max_effort = saturation_effort * (
            1.0 - self.joint_vel / velocity_limit
        )
        max_effort = np.clip(
            max_effort, a_min=0.0, a_max=effort_limit
        )
        min_effort = saturation_effort * (
            -1.0 - self.joint_vel / velocity_limit
        )
        min_effort = np.clip(
            min_effort, a_min=-effort_limit, a_max=0.0
        )


        efforts = np.clip(computed_effort, a_min=min_effort, a_max=max_effort)
        return(efforts[0][np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])])
        
    def GetActions(self):
        
        self.raw_actions = self.ort_sess.run(None, {"obs": self.obs})[0][0, :]
        self.last_actions[0, :] = self.raw_actions[:]

        # self.robot_frame_raw_actions = self.raw_actions[np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])]

        self.actions = self.raw_actions * ACTION_SCALE + self._isaaclab_offset

    # def SetJointTorque(self, joint, tau):
    def SetEffortCmd(self):
        for i in range(12):
            self.low_cmd.motor_cmd[i].q = 0
            self.low_cmd.motor_cmd[i].dq = 0
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = self.ordered_efforts[i] 


        print(self.low_cmd.motor_cmd[0])


    def SetJointAction(self, joint, q, dq = 0, kp=20, kd = 8, tau = 0, z = 0):
        current_pos = self.low_state.motor_state[joint].q
        self.low_cmd.motor_cmd[joint].q = current_pos * z + q * (1 - z)
        self.low_cmd.motor_cmd[joint].dq = dq
        self.low_cmd.motor_cmd[joint].kp = kp
        self.low_cmd.motor_cmd[joint].kd = kd
        self.low_cmd.motor_cmd[joint].tau = tau

    def SetJointsSmooth(self, target_pos, duration, start_pos = None):
        if start_pos is None:
            start_pos = [0.0] * 12
            for i in range(12):
                start_pos[i] = self.low_state.motor_state[i].q
        
        percent = 0
        while percent < 1:
            percent += 1.0 / duration
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - percent) * start_pos[i] + percent * target_pos[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher.Write(self.low_cmd)
            time.sleep(self.dt)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot and the robot is in the lie down position while running this example.")
    input("Press Enter to continue...")

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
