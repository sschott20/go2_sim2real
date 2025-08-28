import numpy as np 

LegID = {
    "FR_0": 0,  # Front right hip
    "FR_1": 1,  # Front right thigh
    "FR_2": 2,  # Front right calf
    "FL_0": 3,
    "FL_1": 4,
    "FL_2": 5,
    "RR_0": 6,
    "RR_1": 7,
    "RR_2": 8,
    "RL_0": 9,
    "RL_1": 10,
    "RL_2": 11,
}

HIGHLEVEL = 0xEE
LOWLEVEL = 0xFF
TRIGERLEVEL = 0xF0
PosStopF = 2.146e9
VelStopF = 16000.0

policy_base_lin_vel = [-0.0164, 0.0025, -0.0038]
policy_base_ang_vel = [0.0083, -0.0052, 0.0288]
policy_projected_gravity = [-0.0361, 0.0035, -0.9993]
policy_velocity_commands = [0.0, 0.0, 0.0]

policy_joint_pos_hip = [-0.0284, -0.0233, 0.0049, 0.005]
policy_joint_pos_thigh = [-0.0469, -0.0556, -0.0298, -0.011]
policy_joint_pos_calf = [0.0314, 0.0869, 0.0008, 0.0112]

policy_joint_vel_hip = [-0.0367, -0.0279, 0.0188, -0.0088]
policy_joint_vel_thigh = [0.0122, 0.0484, -0.0957, -0.0376]
policy_joint_vel_calf = [-0.1123, -0.1061, 0.0247, -0.0143]

policy_actions_hip = [-0.3139, 0.3574, -0.6742, 0.5079]
policy_actions_thigh = [-0.2298, -0.3283, -0.2093, -0.1156]
policy_actions_calf = [0.8549, 1.1967, 0.7898, 0.4889]

# Concatenate all arrays into a single NumPy array
sample_obs = np.concatenate([
    policy_base_lin_vel,
    policy_base_ang_vel,
    policy_projected_gravity,
    policy_velocity_commands,
    policy_joint_pos_hip,
    policy_joint_pos_thigh,
    policy_joint_pos_calf,
    policy_joint_vel_hip,
    policy_joint_vel_thigh,
    policy_joint_vel_calf,
    policy_actions_hip,
    policy_actions_thigh,
    policy_actions_calf
]).reshape(1, -1).astype(np.float32)