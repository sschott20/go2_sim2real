from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
import os



def display(state: LowState_):
    os.system('clear')  # Clear the terminal screen

    print("=== IMU State Data ===\n")

    # Quaternion (w, x, y, z)
    print("Quaternion:           [   w   ][   x   ][   y   ][   z   ]")
    print("                      {:9.3f}{:9.3f}{:9.3f}{:9.3f}".format(
        state.imu_state.quaternion[0],
        state.imu_state.quaternion[1],
        state.imu_state.quaternion[2],
        state.imu_state.quaternion[3]
    ))

    # Euler angles (roll, pitch, yaw)
    print("\nRPY (rad):            [  roll  ][  pitch  ][  yaw  ]")
    print("                      {:9.3f}{:9.3f}{:9.3f}".format(
        state.imu_state.rpy[0],
        state.imu_state.rpy[1],
        state.imu_state.rpy[2]
    ))

    # Gyroscope (x, y, z angular velocity)
    print("\nGyro (rad/s):         [   x   ][   y   ][   z   ]")
    print("                      {:9.3f}{:9.3f}{:9.3f}".format(
        state.imu_state.gyroscope[0],
        state.imu_state.gyroscope[1],
        state.imu_state.gyroscope[2]
    ))

    # Accelerometer (x, y, z acceleration)
    print("\nAccel (m/s²):         [   x   ][   y   ][   z   ]")
    print("                      {:9.3f}{:9.3f}{:9.3f}".format(
        state.imu_state.accelerometer[0],
        state.imu_state.accelerometer[1],
        state.imu_state.accelerometer[2]
    ))

    print("\n=== Joint Positions (rad) ===\n")
    print("                      [  hip  ][ thigh][ calf  ]")

    # Joint positions for all 12 joints (3 per leg)
    print("Front Left:           {:9.3f}{:9.3f}{:9.3f}".format(
        state.motor_state[0].q,
        state.motor_state[1].q,
        state.motor_state[2].q
    ))
    print("Front Right:          {:9.3f}{:9.3f}{:9.3f}".format(
        state.motor_state[3].q,
        state.motor_state[4].q,
        state.motor_state[5].q
    ))
    print("Rear Left:            {:9.3f}{:9.3f}{:9.3f}".format(
        state.motor_state[6].q,
        state.motor_state[7].q,
        state.motor_state[8].q
    ))
    print("Rear Right:           {:9.3f}{:9.3f}{:9.3f}".format(
        state.motor_state[9].q,
        state.motor_state[10].q,
        state.motor_state[11].q
    ))

    print("\n=== Foot Forces (N) ===\n")

    # Foot forces for all 4 feet
    print("Front Left:           {:9.1f}".format(state.foot_force[0]))
    print("Front Right:          {:9.1f}".format(state.foot_force[1]))
    print("Rear Left:            {:9.1f}".format(state.foot_force[2]))
    print("Rear Right:           {:9.1f}".format(state.foot_force[3]))

