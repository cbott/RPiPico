# settings.py
# Constants used across several NNet files to define robot behavior and training
from typing import Iterable

# Robot segment lengths [mm]
ROBOT_L1 = 120
ROBOT_L2 = 115
ROBOT_L3 = 58

# Robot useable (for NNet training purposes) operating volume [mm]
ROBOT_RANGE = [
    (-200, 200),
    (0, 200),
    (0, 200)
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


def scale_values(values: Iterable[float],
                 input_ranges: list[tuple[float, float]],
                 output_ranges: list[tuple[float, float]]) -> list[float]:
    """
    Map provided values from the input range to the output range using a linear scaling

    values: list of "real" values which have a range that falls within the corresponding element of input_ranges
    input_ranges: list of tuples, each tuple representing the possible range of the corresponding coordinate
    output_ranges: list of tuples, each tuple representing the range to scale that element to
    """
    result = []
    for i, value in enumerate(values):
        (input_min, input_max) = input_ranges[i]
        (output_min, output_max) = output_ranges[i]
        new_value = (value - input_min) / (input_max - input_min) * (output_max - output_min) + output_min
        result.append(new_value)
    return result
