# calibrate motors 3/19/25

# Tz = 0.25, 0.5, 0.75, 1.0 
# Tx = 0
# Ty = 0

# kinematics library for ballbot rotation

from maths.ballbot_kinematics import compute_ball_rotation

# kinematics library for ballbot motor torques (do not use yet)

from maths.ballbot_kinetic import compute_motor_torques