from math import atan2, cos, pi, sin
import numpy as np
import sys
from unittest.mock import MagicMock

sys.modules['machine'] = MagicMock()
sys.modules['utime'] = MagicMock()
from ax12 import Robot

# Wolfram alpha DH matrices
# {{cos\(40)a\(41),0,sin\(40)a\(41),0},{sin\(40)a\(41),0,-cos\(40)a\(41),0},{0,1,0,0},{0,0,0,1}}{{cos\(40)b\(41),-sin\(40)b\(41),0,l*cos\(40)b\(41)},{sin\(40)b\(41),cos\(40)b\(41),0,l*sin\(40)b\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)c\(41),-sin\(40)c\(41),0,m*cos\(40)c\(41)},{sin\(40)c\(41),cos\(40)c\(41),0,m*sin\(40)c\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)d\(41),-sin\(40)d\(41),0,n*cos\(40)d\(41)},{sin\(40)d\(41),cos\(40)d\(41),0,n*sin\(40)d\(41)},{0,0,1,0},{0,0,0,1}}

def get_dh_array(d, theta, r, alpha):
    return np.array(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]]
        )

# # Transformation matrices
# r01 = get_dh_array(0, t0, 0, np.pi / 2)
# r12 = get_dh_array(0, t1, l1, 0)
# r23 = get_dh_array(0, t2, l2, 0)
# r34 = get_dh_array(0, t3, l3, 0)

# transform = r01 @ r12 @ r23 @ r34

# # print(np.round(transform[0:3, 3], 2))
# print(np.round(transform, 2))

def get_forward(theta0, theta1, theta2, theta3):
    r01 = get_dh_array(0, theta0, 0, np.pi / 2)
    r12 = get_dh_array(0, theta1, Robot.L1, 0)
    r23 = get_dh_array(0, theta2, Robot.L2, 0)
    r34 = get_dh_array(0, theta3, Robot.L3, 0)

    r04 = r01 @ r12 @ r23 @ r34
    return r04[0:3, 3]


def check_inverse_math(x, y, z):
    print(f"Target position ({x}, {y}, {z})")
    theta0, theta1, theta2, theta3 = Robot.inverse_kinematics(x, y, z)
    print(f"Joint angles ({theta0}, {theta1}, {theta2}, {theta3})")
    real_pos = get_forward(theta0, theta1, theta2, theta3)
    real_x = real_pos[0]
    real_y = real_pos[1]
    real_z = real_pos[2]
    print("Actual position", real_pos)
    dx = x - real_x
    dy = y - real_y
    dz = z - real_z
    print(f"Error {round((dx**2 + dy**2 + dz**2)**0.5, 4)} ({dx}, {dy}, {dz})")

check_inverse_math(144, 10, 137)
