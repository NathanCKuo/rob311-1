"""
Ball-bot Kinetic Module
==========================

This program takes the ball-bot body frame torque Tx, Ty, Tz along x, y, z axes respectively
and covert it to the torques of each motor T1, T2, T3.

Authors: Yilin Ma
Hybrid Dynamic Robotics Lab
"""

import numpy as np

# Constants
FREQ = 200  # Frequency in Hz
DT = 1 / FREQ  # Time step
RK = 0.1192 # Ball radius (m)
RW = 0.0488  # Wheel radius (m)
ALPHA = np.deg2rad(45)  # Motor inclination angle (radians)

# Function to convert frame torque to motor torque
def compute_motor_torques(Tx, Ty, Tz):
    '''
    Parameters:
    ----------
    Tx: Torque along x-axis
    Ty: Torque along y-axis
    Tz: Torque along z-axis

    Returns:
    --------
            Ty
            T1
            |
            |
            |
            . _ _ _ _ Tx
           / \
          /   \
         /     \
        /       \
       T2       T3

    T1: Motor Torque 1
    T2: Motor Torque 2
    T3: Motor Torque 3
    '''
# -------------------------------------------------------------------   
# YOUR
# CODE
# GOES
# HERE 



    T1 = (1 / 3) * (Tz - (2 * Ty) / np.cos(ALPHA))
    T2 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (-np.sqrt(3) * Tx + Ty))
    T3 = (1 / 3) * (Tz + (1 / np.cos(ALPHA)) * (np.sqrt(3) * Tx + Ty))

    T1 = np.clip(T1, -1, 1)
    T2 = np.clip(T2, -1, 1)
    T3 = np.clip(T3, -1, 1)

# -------------------------------------------------------------------
    return T1, T2, T3
