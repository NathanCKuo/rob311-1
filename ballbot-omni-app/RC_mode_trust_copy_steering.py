"""
Steering Controller for Ball-Bot on the Floor

This script integrates:
1. PS4 controller inputs to dynamically adjust the torques applied to the motors of a ball-bot.
2. Real-time sensor readings from the robot to monitor the system's state.
3. Data logging to store relevant information (e.g., sensor readings, torque commands) for analysis.

Key Features:
- PS4 controller interface to adjust torques dynamically.
- Computes motor torques using inputs and the `compute_motor_torques` function from `ball_kinematics.py`.
- Reads sensor data (e.g., IMU angles, motor rotations) and logs them in a structured file for debugging and analysis.

Requirements:
- PS4 controller connected via `/dev/input/js0`.
- Serial communication with the ball-bot for state updates and command transmission.

Author: Yilin Ma, and Wenzhe Tong
Hybrid Dynamic Robotics Lab
Date: January 2025
Edited by Nathan Kuo and Lucia Lee (team name: trust)
Edit data: 03/19/2025
"""

# Default Library
import sys
import threading
import time
import numpy as np
from threading import Thread
from src.loop import SoftRealtimeLoop
from src.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from src.SerialProtocol.protocol import SerialProtocol
from src.DataLogger import dataLogger
from collections import deque

# Library for PS4 Controller
from src.ps4_controller_api import PS4InputHandler

# Kinematic library, may change at different stage
#from src.bot_kinematic import compute_robot_motor_torques # only for floor rc mold

# kinematics library for ballbot rotation

from maths.ballbot_kinematics import compute_ball_rotation

# kinematics library for ballbot motor torques (do not use yet)

from maths.ballbot_kinetic import compute_motor_torques

# Constants for the control loop
FREQ = 200  # Frequency of control loop in Hz
DT = 1 / FREQ  # Time step for each iteration in seconds
JOYSTICK_SCALE = 32767  # Scale factor for normalizing joystick values


# Torque scaling factors
"""
    These constants define the maximum torque values that can be applied along
    each axis. They scale the raw input from the PS4 controller to a range
    suitable for controlling the robot's motors. These values depend on the
    root's hardware capabilities and desired performance.

 TX_MAX: Maximum torque along the x-axis (sideways motion).
    - Selected based on the motor's torque limits and the desired response speed.
    - Suggested range: TBD

 TY_MAX: Maximum torque along the y-axis (forward/backward motion).
    - Similar to TX_MAX, this value ensures the robot does not exceed its safe
      operating torque while still providing sufficient responsiveness.
    - Suggested range: TBD

 TZ_MAX: Maximum torque along the z-axis (rotational/yaw motion).
    - Typically smaller than TX_MAX and TY_MAX because yaw adjustments require
      less torque compared to linear motion.
    - Suggested range: TBD
"""

TX_MAX = 0.5  # Maximum torque along x-axis
TY_MAX = 0.5 # Maximum torque along y-axis
TZ_MAX = 0.5  # Maximum torque along z-axis


if __name__ == "__main__":
    # === Controller Initialization ===
    # Create an instance of the PS4 controller handler
    controller = PS4InputHandler(interface="/dev/input/js0")

    # Start a separate thread to listen for controller inputs
    controller_thread = threading.Thread(target=controller.listen, args=(10,))
    controller_thread.daemon = True  # Ensures the thread stops with the main program
    controller_thread.start()

    print("PS4 Controller is active. Use thumbsticks and triggers to control torque.")

    # === Serial Communication Initialization ===
    # Initialize the serial communication protocol
    ser_dev = SerialProtocol()
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]
    
    # Start a separate thread for reading serial data
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # === Data Logging Initialization ===
    # Prompt user for trial number and create a data logger
    trial_num = int(input("Trial Number? "))
    filename = f"controller_data_trial_{trial_num}.txt"
    dl = dataLogger(filename)
    data_name = ['i', 't_now', 'roll', 'pitch', 'yaw', 'Tx', 'Ty', 'Tz', 'T1', 'T2', 'T3', 'psi_1', 'psi_2','psi_3', 'Tpx', 'Tpy', 'Tix', 'Tiy', 'Tdx', 'Tdy', 'error_x', 'error_y']
    dl.appendData(data_name)

    # === Command and State Structures ===
    # Define command structure for controlling motors
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    commands['start'] = 1.0  # Activate motors

    # Initialize state structure for reading sensors
    states = np.zeros(1, dtype=mo_states_dtype)[0]

    # Allow communication to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)

    print("Starting steering control loop...")
    # === Main Control Loop ===
    i = 0  # Iteration counter

    errorx_prev = 0.0
    errory_prev = 0.0
    errorz_prev = 0.0
    sumx = 0.0
    sumy = 0.0
    t_start = time.time()


    de = deque()  

    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Read sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]

            # === Controller Input ===
            # Fetch signals from the PS4 controller
            signals = controller.get_signals()

            # Map joystick and trigger inputs to torques
            """
                This is a sample implementation of mapping controller inputs to 
                torque commands (Tx, Ty, Tz):
                    - Tx: Controlled by the horizontal movement of the left thumbstick (x-axis).
                    - Ty: Controlled by the vertical movement of the left thumbstick (y-axis).
                    - Tz: Controlled by the horizontal movement of the right thumbstick (x-axis).
            
                 You can modify this mapping to suit your project goals. 
                 
                 Potential Usage:
                    "left_thumbstick_x": 0.0,    Left thumbstick horizontal (e.g., strafe control)
                    "left_thumbstick_y": 0.0,    Left thumbstick vertical (e.g., forward/backward)
                    "right_thumbstick_x": 0.0,   Right thumbstick horizontal (e.g., yaw control)
                    "right_thumbstick_y": 0.0,   Right thumbstick vertical (e.g., pitch control)
                    "trigger_L2": 0.0,           Left trigger (e.g., brake or reduce thrust)
                    "trigger_R2": 0.0,           Right trigger (e.g., accelerate or increase thrust)
                    "shoulder_L1": 0,            Left shoulder button (e.g., discrete decrement)
                    "shoulder_R1": 0,            Right shoulder button (e.g., discrete increment)
            """
            # ---------------------------------------------------------------
            # Your Code Goes Here
            # CONTROLLER DEADZONES
            left_x = signals["left_thumbstick_x"] 
            left_y = signals["left_thumbstick_y"] 
            # right_x = signals["right_thumbstick_x"] 
            # right_y = signals["right_thumbstick_y"] 
            # trigger_L2 = signals["trigger_L2"]
            # trigger_R2 = signals["trigger_R2"]
            # shoulder_L1 = signals["shoulder_L1"]
            # shoulder_R1 = signals["shoulder_R1"]
            if (abs(left_x) < 0.02) :
                left_x = 0
            if (abs(left_y) < 0.02) :
                left_y = 0
            # if (abs(right_x) < 0.02) :
            #     right_x = 0
            # if (abs(right_y) < 0.02) :
            #     right_y = 0
            
            desired_angle_y = -0.00
            desired_angle_x = -0.0
            desired_angle_z = 0.0
            # resting_angle = 0.0
            roll = states['theta_roll'] # Roll angle (x-axis)
            pitch = states['theta_pitch']  # Pitch angle (y-axis)
            yaw = states['theta_yaw'] #Yaw angle (z-axis)
            # Extract sensor readings
            psi_1 = states['psi_1']  # Motor 1 rotation
            psi_2 = states['psi_2']  # Motor 2 rotation
            psi_3 = states['psi_3']  # Motor 3 rotation
            dpsi1 = states['dpsi_1']
            dpsi2 = states['dpsi_2']
            dpsi3 = states['dpsi_3']

            t_now = time.time() - t_start  # Elapsed

            T_out_px = 0
            T_out_py = 0
            T_out_ix = 0
            T_out_iy = 0
            T_out_dx = 0
            T_out_dy = 0
            errorx = 0
            errory = 0
            error_xd = 0

            d_filtered_x = 0.0
            d_filtered_y = 0.0
            alpha = 0.025 

            if(t_now > 15.00):
                # CONTROLLER DEADZONES
                left_x = signals["left_thumbstick_x"]
                left_y = signals["left_thumbstick_y"]
                # right_x = signals["right_thumbstick_x"]
                # right_y = signals["right_thumbstick_y"]
                # trigger_L2 = signals["trigger_L2"]
                # trigger_R2 = signals["trigg`er_R2"]
                # shoulder_L1 = signals["shoulder_L1"]
                # shoulder_R1 = signals["shoulder_R1"]
                if (abs(left_x < 0.1)) :
                    left_x = 0
                if (abs(left_y < 0.1)) :
                    left_y = 0
                # if (abs(right_x < 0.1)) :
                #     right_x = 0
                # if (abs(right_y < 0.1)) :
                #     right_y = 0
                # XZ-plane steering controller
                max_angle_x = 0.05 # max lean angle for xz plane
                max_angle_y = 0.08 # max lean angle for yz plane
                max_angle = 0.1
                desired_angle_roll = left_y * max_angle_x -0.0112
                # YZ-plane steering controller
                desired_angle_pitch = left_x * max_angle_y #left_y * max_angle_y
                
                # Rotation controller
                T_out_z = 0


                errorx = desired_angle_roll - roll #- 0.005
                errory = desired_angle_pitch - pitch 
                errorz = desired_angle_z - yaw 

                if (abs(errory) > 0.005) :
                    sumy += errory
                if (abs(errorx) > 0.005) :
                    sumx += errorx

                # XZ-plane balance controller
                Kp_x = 7.9
                Ki_x = 0.01
                Kd_x = 0.6

                # xz filtering
                d_raw_x = (errorx - errorx_prev) / DT
                d_filtered_x = alpha * d_raw_x + (1 - alpha) * d_filtered_x
                T_out_dx = Kd_x * d_filtered_x

                T_out_px = Kp_x * -errorx
                T_out_ix = Ki_x * -(sumx) * DT
                #T_out_dx = Kd_x * ((error_xd - errorx_prev)/DT)
                T_out_x = T_out_px + T_out_dx + T_out_ix

                # YZ-plane balance controller
                Kp_y = 7.7
                Ki_y = 0.015
                Kd_y = 0.6
                # if (pitch < 0):
                #     kd_y = .155

                # yz filtering
                d_raw_y = (errory - errory_prev) / DT
                d_filtered_y = alpha * d_raw_y + (1 - alpha) * d_filtered_y
                T_out_dy = Kd_y * d_filtered_y

                T_out_py = Kp_y * errory
                T_out_iy = Ki_y * -(sumy) * DT
                T_out_dy = Kd_y * ((errory - errory_prev)/DT)           
                T_out_y = T_out_py + T_out_dy + T_out_iy
                
                errorx_prev = errorx
                errory_prev = errory
                #errorz_prev = errorz

                

                # Tx = signals["left_thumbstick_x"] * TX_MAX
                # Ty = signals["left_thumbstick_y"] * TY_MAX
                # Tz = signals["right_thumbstick_x"] * TZ_MAX
                
                Tx = T_out_x
                Ty = T_out_y
                Tz = T_out_z  
            else:   
                Tx = 0
                Ty = 0
                Tz = 0

            # ---------------------------------------------------------------

            # === Torque Computation ===
            T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz) # for floor test (originally compute_robot_motor_torques from src)

            
            # Send computed torques to the robot
            commands['motor_1_duty'] = T1
            commands['motor_2_duty'] = T2
            commands['motor_3_duty'] = T3
            ser_dev.send_topic_data(101, commands)

            # === Data Logging ===
            
            i+=1

            # Log data to the file,
            data = [i, t_now, roll, pitch, yaw, Tx, Ty, Tz, T1, T2, T3, psi_1, psi_2, psi_3, T_out_px, T_out_py,
            T_out_ix, T_out_iy, T_out_dx, T_out_dy, errorx, errory, error_xd]
            dl.appendData(data)

            print(
                f"Time: {t_now:.2f}s | Tx: {Tx:.4f}, Ty: {Ty:.4f}, Tz: {Tz:.4f} | "
                f"T1: {T1:.4f}, T2: {T2:.4f}, T3: {T3:.4f} | "
                f"Roll: {roll:.4f}, Pitch: {pitch:.4f}, Yaw: {yaw:.2f} | "
                f"Left_x: {left_x}, Left_y: {left_y}"
            )

        except KeyError:
            print("Waiting for sensor data...")

    # === Shutdown ===
    print("Saving data...")
    dl.writeOut()  # Write logged data to the file

    print("Shutting down motors...")
    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)
