"""
Ball-bot Kinematics Module
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

# ---------------------------------------------------------------------------

def compute_ball_rotation(psi1, psi2, psi3):
    """
    Computes the ball's rotational velocities (phi_x, phi_y, phi_z)
    given the wheel velocities (psi1, psi2, psi3).
    
    Parameters:
    -----------
    psi1, psi2, psi3 : float
        Wheel angular velocities
    
    Returns:
    --------
    phi_x, phi_y, phi_z : float
        Ball rotational velocities
    """
    phi_x = (2 / 3) * (RW / RK) * (psi2 - psi3)
    phi_y = (np.sqrt(2) / 3) * (RW / RK) * (-2 * psi1 + psi2 + psi3)
    phi_z = (np.sqrt(2) / 3) * (RW / RK) * (psi1 + psi2 + psi3)
    
    return phi_x, phi_y, phi_z