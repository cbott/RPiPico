# from math import atan2, cos, pi, sin
# import numpy as np
# import sys
# from typing import List
# from unittest.mock import MagicMock

# sys.modules['machine'] = MagicMock()
# sys.modules['utime'] = MagicMock()
# from ax12 import Pose, Robot

# # Wolfram alpha DH matrices
# # {{cos\(40)a\(41),0,sin\(40)a\(41),0},{sin\(40)a\(41),0,-cos\(40)a\(41),0},{0,1,0,0},{0,0,0,1}}{{cos\(40)b\(41),-sin\(40)b\(41),0,l*cos\(40)b\(41)},{sin\(40)b\(41),cos\(40)b\(41),0,l*sin\(40)b\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)c\(41),-sin\(40)c\(41),0,m*cos\(40)c\(41)},{sin\(40)c\(41),cos\(40)c\(41),0,m*sin\(40)c\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)d\(41),-sin\(40)d\(41),0,n*cos\(40)d\(41)},{sin\(40)d\(41),cos\(40)d\(41),0,n*sin\(40)d\(41)},{0,0,1,0},{0,0,0,1}}

# def get_dh_array(d, theta, r, alpha):
#     return np.array(
#         [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
#          [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
#          [0, np.sin(alpha), np.cos(alpha), d],
#          [0, 0, 0, 1]]
#         )

# # # Transformation matrices
# # r01 = get_dh_array(0, t0, 0, np.pi / 2)
# # r12 = get_dh_array(0, t1, l1, 0)
# # r23 = get_dh_array(0, t2, l2, 0)
# # r34 = get_dh_array(0, t3, l3, 0)
# # transform = r01 @ r12 @ r23 @ r34


# def get_forward(theta0, *args):
#     """ Get forward kinematic position of n joint angles (radians) """
#     if len(args) > 3:
#         raise ValueError(f'Invalid number of joint angles provided. Received joint angles of {theta0}, {args}')

#     # Transformation from joint 0 through joint n
#     r0n = get_dh_array(0, theta0, 0, np.pi / 2)
#     joints = [Robot.L1, Robot.L2, Robot.L3]

#     for i, theta in enumerate(args):
#         r = get_dh_array(0, theta, joints[i], 0)
#         r0n = r0n @ r

#     return r0n[0:3, 3]


# def unit_vector(vector: List) -> List:
#     magnitude = sum([n**2 for n in vector])**0.5
#     return [n/magnitude for n in vector]


# # def fabrik(x, y, z) -> Pose:
# #     """ Numerical method """
# #     L1 = 120
# #     L2 = 115

# #     target_p = [x, y, z]

# #     current_theta0 = 0
# #     current_theta1 = 0
# #     current_theta2 = 0

# #     r01 = get_dh_array(0, current_theta0, 0, np.pi / 2)
# #     r12 = get_dh_array(0, current_theta1, Robot.L1, 0)
# #     r23 = get_dh_array(0, current_theta2, Robot.L2, 0)

# #     current_p3 = (r01 @ r12 @ r23)[0:3, 3]
# #     current_p2 = (r01 @ r12)[0:3, 3]

# #     for iteration in range(5):
# #         # Unit vector from target to current joint 2
# #         v_t_2 = unit_vector([current_p2[i] - target_p[i] for i in range(3)])

# #         candidate_p2 = [target_p[i] + v_t_2[i] * L2 for i in range(3)]

# #         # Unit vector from base to candidate joint 2 position
# #         v_b_c2 = unit_vector(candidate_p2)
# #         achievable_p2 = [v_b_c2[i] * L1 for i in range(3)]

# #         # Unit vector from achievable joint 2 position to target
# #         # These 2 things are only useful for calculating error I guess?
# #         v_a2_t = unit_vector([target_p[i] - achievable_p2[i] for i in range(3)])
# #         achievable_p3 = [achievable_p2[i] + v_a2_t[i] * L2 for i in range(3)]
# #         # Caluclate error or something

# #         current_p2 = achievable_p2

#     # At this point we have a pretty accurate [xyz] coordinate for each joint position, now we need their angles
#     # theta0 =
#     # theta1 = ?
#     # theta2 = ?
#     # theta3 = defined by theta1 and theta2 to meet end effector requirement

# def get_inverse(x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False) -> Pose:
#     """ Mirror of Robot class method to make importing easier """
#     return Robot.inverse_kinematics(x, y, z, end_angle, invert_base, invert_elbow)


# def check_inverse_math(x, y, z):
#     print(f"Target position ({x}, {y}, {z})")
#     theta0, theta1, theta2, theta3 = get_inverse(x, y, z)
#     print(f"Joint angles ({theta0}, {theta1}, {theta2}, {theta3})")
#     real_pos = get_forward(theta0, theta1, theta2, theta3)
#     real_x = real_pos[0]
#     real_y = real_pos[1]
#     real_z = real_pos[2]
#     print("Actual position", real_pos)
#     dx = x - real_x
#     dy = y - real_y
#     dz = z - real_z
#     print(f"Error {round((dx**2 + dy**2 + dz**2)**0.5, 4)} ({dx}, {dy}, {dz})")


# if __name__ == '__main__':
#     check_inverse_math(144, 10, 137)
