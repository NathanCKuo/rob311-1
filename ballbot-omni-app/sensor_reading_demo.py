"""
ROB 311 - Ball-bot Sensor Reading Demo
This program demonstrates a soft real-time control loop for the ball-bot project.
It ensures loop timing using a SoftRealtimeLoop and manages communication with the hardware via serial.

Authors: Yilin Ma, Senthur Raj, Gray Thomas, Yves Nazon, Xiaonan Huang, and Elliott Rouse 
Neurobionics Lab / Locomotor Control Lab / Hybrid Dynamic Robtoics Lab
"""

import sys
import threading
import time
import numpy as np
from threading import Thread
from src.Messages.message_defs import mo_states_dtype, mo_cmds_dtype
from src.SerialProtocol.protocol import SerialProtocol
from src.loop import SoftRealtimeLoop
from src.DataLogger import dataLogger

# Control loop frequency (Hz)
FREQ = 200
DT = 1 / FREQ  # Loop time step in seconds

def register_topics(ser_dev: SerialProtocol):
    """
    Register the data topics for serial communication.
    This maps specific message IDs to their corresponding serialization/deserialization functions.
    
    :param ser_dev: Instance of SerialProtocol to manage communication
    """
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

if __name__ == "__main__":
    # Prompt user for trial number to log data
    trial_num = int(input('Trial Number? '))
    filename = f'ROB311_Test{trial_num}.txt'
    dl = dataLogger(filename)  # Initialize data logger

    # Initialize serial communication with the hardware
    ser_dev = SerialProtocol()
    register_topics(ser_dev)  # Register message types

    # Start serial read thread to handle incoming data
    #serial_read_thread = Thread(target=ser_dev.read_loop, args=(), daemon=True)
    #serial_read_thread.start()
    serial_read_thread = threading.Thread(target=SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # Define command and state structures
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]  # Initialize command structure
    states = np.zeros(1, dtype=mo_states_dtype)[0]  # Initialize state structure

    # Set initial command to start the system
    commands['start'] = 1.0

    # Allow some time for serial communication to synchronize
    time.sleep(1.0)
    
    # Send initial command to the robot
    ser_dev.send_topic_data(101, commands)

    print('Beginning program!')
    i = 0  # Loop iteration counter

    # Main control loop using soft real-time scheduling
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            # Retrieve the latest sensor data from the robot
            states = ser_dev.get_cur_topic_data(121)[0]
        except KeyError:
            continue  # If no data is available, skip this iteration

        if i == 0:
            print('Finished calibration\nStarting loop...')
            t_start = time.time()  # Record start time

        i += 1  # Increment loop counter
        t_now = time.time() - t_start  # Compute elapsed time

        dpsi[0] = states['dpsi_1']
        dpsi[1] = states['dpsi_2']
        dpsi[2] = states['dpsi_3']

        ser_dev.send_topic_data(101, commands)

        # Extract sensor readings of the bot's orientation
        theta_x = states['theta_roll']  # Roll angle
        theta_y = states['theta_pitch']  # Pitch angle

        # Construct data array for logging
        # Construct the data matrix for saving - you can add more variables by replicating the format below
        # Data structure: 
        # [iteration index, time elapsed, roll angle, pitch angle, angular velocity 1, angular velocity 2, angular velocity 3]
            # iteration index: Keeps track of loop cycles
            # time elapsed: Time since the start of data collection
            # roll angle: Robot's roll angle from IMU
            # pitch angle: Robot's pitch angle from IMU
            # angular velocity 1: Angular velocity measurement from wheel 1
            # angular velocity 2: Angular velocity measurement from wheel 2
            # angular velocity 3: Angular velocity measurement from wheel 3
        data = [i, t_now, theta_x, theta_y, dpsi[0], dpsi[1], dpsi[2]]
        dl.appendData(data)

    
    print("Saving data...")
    dl.writeOut()
    print("Resetting Motor Commands.")
    commands['start'] = 0.0
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    ser_dev.send_topic_data(101, commands)