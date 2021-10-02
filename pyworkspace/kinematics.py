import numpy as np

# Beam lengths
l1 = 120
l2 = 80
l3 = 40

# Servo Angles
t0 = 0
t1 = np.pi / 2
t2 = -2.84
t3 = -0.8

# 0
# 0
# 0


def get_dh_array(d, theta, r, alpha):
    return np.array(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]]
        )

# Transformation matrices
r01 = get_dh_array(0, t0, 0, np.pi / 2)
r12 = get_dh_array(0, t1, l1, 0)
r23 = get_dh_array(0, t2, l2, 0)
r34 = get_dh_array(0, t3, l3, 0)

transform = r01 @ r12 @ r23 @ r34

# print(np.round(transform[0:3, 3], 2))
print(np.round(transform, 2))


# {{cos\(40)a\(41),0,sin\(40)a\(41),0},{sin\(40)a\(41),0,-cos\(40)a\(41),0},{0,1,0,0},{0,0,0,1}}{{cos\(40)b\(41),-sin\(40)b\(41),0,l*cos\(40)b\(41)},{sin\(40)b\(41),cos\(40)b\(41),0,l*sin\(40)b\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)c\(41),-sin\(40)c\(41),0,m*cos\(40)c\(41)},{sin\(40)c\(41),cos\(40)c\(41),0,m*sin\(40)c\(41)},{0,0,1,0},{0,0,0,1}}{{cos\(40)d\(41),-sin\(40)d\(41),0,n*cos\(40)d\(41)},{sin\(40)d\(41),cos\(40)d\(41),0,n*sin\(40)d\(41)},{0,0,1,0},{0,0,0,1}}
