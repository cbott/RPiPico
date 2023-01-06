# settings.py
# Constants used across several NNet files to define robot behavior and training

# Robot segment lengths [mm]
ROBOT_L1 = 120
ROBOT_L2 = 115
ROBOT_L3 = 58

# Robot useable (for NNet training purposes) operating volume [mm]
ROBOT_RANGE = [
    (-200.0, 200.0),
    (0.0, 200.0),
    (0.0, 200.0)
]

# Robot servo limits [radians]
JOINT_LIMITS = [
    (-1.017, 4.16),
    (-0.04, 3.148),
    (-2.469, 2.36)
]

# NNet input and output ranges, one for each node
# defining how much to scale from reality to improve model performance

# NNET_INPUT_RANGE = ROBOT_RANGE
NNET_INPUT_RANGE = [
    (-1.0, 1.0),
    (-1.0, 1.0),
    (-1.0, 1.0)
]

# NNET_OUTPUT_RANGE = JOINT_LIMITS
NNET_OUTPUT_RANGE = [  # Scaled version of angles
    (-1.0, 1.0),
    (-1.0, 1.0),
    (-1.0, 1.0)
]
