import numpy as np
from pathlib import Path
import pickle
import random
import sys
from typing import List, Tuple, Optional
from unittest.mock import MagicMock

sys.modules['machine'] = MagicMock()
sys.modules['utime'] = MagicMock()
from ax12 import Robot

from kinematics import get_forward, get_inverse

JOINT_LIMITS = [
    (-1.017, 4.16),
    (-0.04, 3.148),
    (-2.469, 2.36)
]

OUTPUT_SPACE = [
    (-200, 200),
    (0, 200),
    (0 - Robot.L3, 200 - Robot.L3)
]

def generate_position() -> Tuple[List[float], List[float]]:
    """ Return a tuple of ([position], [angles]) for a random robot pose """
    # Generate a random pose within the robot's range of motion
    angles = [random.uniform(*limits) for limits in JOINT_LIMITS]
    # Calculate end effector positon for this pose
    end_position = list(get_forward(*angles))
    return (end_position, angles)

def generate_position_constrained() -> Optional[Tuple[List[float], List[float]]]:
    """
    Return a tuple of ([position], [angles]) for a random robot pose
    using inverse kinematic method to ensure each position has a unique input set
    (first 3 joints only)
    """
    # Choose a random point in achievable space
    coordinates = [random.uniform(*limits) for limits in OUTPUT_SPACE]
    try:
        theta0, theta1, theta2, _ = get_inverse(coordinates[0], coordinates[1], coordinates[2], -np.pi / 2)
        # Account for joint3 introducing offset
        angles = [theta0, theta1, theta2]
        for i in range(3):
            if angles[i] < JOINT_LIMITS[i][0] or angles[i] > JOINT_LIMITS[i][1]:
                raise ValueError
        three_joint_coordinates = [coordinates[0], coordinates[1], coordinates[2] + Robot.L3]
        return (three_joint_coordinates, angles)

    except ValueError:
        pass

    # print(f'Unable to reach coorinates {coordinates}')
    return None


def save_position_samples(n_samples: int, save_file: Path) -> None:
    # samples = [generate_position() for i in range(n_samples)]
    samples = []
    while len(samples) < n_samples:
        s = generate_position_constrained()
        if s is not None:
            samples.append(s)

    with open(save_file, 'wb') as f:  # type: ignore
        pickle.dump(samples, f)


def save_multiple_sample_groups(n_files: int, samples_per_file: int, save_directory: str):
    for i in range(n_files):
        filename = f'positiondata_{samples_per_file}_{i}.dat'
        save_position_samples(samples_per_file, Path(save_directory, filename))
    print(f'Saved {n_files} sample files of size {samples_per_file} to {save_directory}/')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise ValueError(f'Position generator takes 3 arguments: n_files, samples_per_file, save_directory')
    save_multiple_sample_groups(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
