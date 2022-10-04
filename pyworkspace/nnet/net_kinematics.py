from math import atan2, cos, pi, sin
import numpy as np
from typing import Tuple

# Robot segment lengths [mm]
ROBOT_L1 = 120
ROBOT_L2 = 115
ROBOT_L3 = 58

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
def get_forward(theta0, *args):
    """ Get forward kinematic position of n joint angles (radians) """
    if len(args) > 3:
        raise ValueError(f'Invalid number of joint angles provided. Received joint angles of {theta0}, {args}')

    # Transformation from joint 0 through joint n
    r0n = get_dh_array(0, theta0, 0, np.pi / 2)
    joints = [ROBOT_L1, ROBOT_L2, ROBOT_L3]

    for i, theta in enumerate(args):
        r = get_dh_array(0, theta, joints[i], 0)
        r0n = r0n @ r

    return r0n[0:3, 3]


def get_inverse(x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False) -> Tuple[float, float, float, float]:
    """ Determine necessary joint angles to reach a position """
    # Solve for joint 0 angle
    theta0 = atan2(y, x)
    if invert_base:
        # When we want to "reach over backwards" the base points the opposite direction of the desired point
        theta0 += pi
    # Constrain base to the valid servo range of -pi/3 to 8pi/6
    # We may still end up in an invalid range, caller must ensure this is not the case
    if theta0 < -pi/3:
        theta0 += 2*pi
    if theta0 > 8*pi/6:
        theta0 -= 2*pi

    # Translate end effector coordinates to joint 3 coordinates
    d_z = ROBOT_L3 * sin(end_angle)
    d_r = ROBOT_L3 * cos(end_angle)
    x = x - d_r * cos(theta0)
    y = y - d_r * sin(theta0)
    z = z - d_z

    # Solve for joint 2 angle
    dist_squared = x**2 + y**2 + z**2
    # solve for cos(theta2) using law of cosines
    cos_theta2 = (dist_squared - ROBOT_L1**2 - ROBOT_L2**2) / (2 * ROBOT_L1 * ROBOT_L2)
    if not -1 <= cos_theta2 <= 1:
        raise ValueError("Target position {} not achievable".format((x, y, z)))
    sin_theta2 = -(1 - cos_theta2**2)**0.5
    if invert_elbow:
        sin_theta2 *= -1
    theta2 = atan2(sin_theta2, cos_theta2)

    # Solve for joint 1 angle
    r = (x**2 + y**2)**0.5
    alpha = atan2(z, r)
    beta = atan2(ROBOT_L2 * sin(theta2), ROBOT_L2 * cos(theta2) + ROBOT_L1)
    theta1 = alpha - beta

    # Make corrections for flipped base
    if invert_base:
        # Base has been set to face away from the target point, reflect remaining joints about z-r plane
        theta1 = pi - theta1
        theta2 *= -1

    # Solve for joint 3 angle
    # TODO: make this play nice with inverted base
    theta3 = end_angle - theta1 - theta2
    # TODO: do this with mod math
    if theta3 < -pi:
        theta3 += 2*pi
    if theta3 > pi:
        theta3 -= 2*pi

    return (theta0, theta1, theta2, theta3)
