# position_generator.py
# Create training and testing data for robot neural network
import pickle
import random
import sys
from pathlib import Path
from settings import *

import numpy as np

import net_kinematics

# def generate_position() -> tuple[list[float], list[float]]:
#     """ Return a tuple of ([position], [angles]) for a random robot pose """
#     # Generate a random pose within the robot's range of motion
#     angles = [random.uniform(*limits) for limits in JOINT_LIMITS]
#     # Calculate end effector positon for this pose
#     end_position = list(net_kinematics.get_forward(*angles))
#     return (end_position, angles)


def generate_position_constrained() -> tuple[list[float], list[float]]:
    """
    Return a tuple of ([position], [angles]) for a random robot pose
    using inverse kinematic method to ensure each position has a unique input set
    (first 3 joints only)
    """
    three_joint_coordinates = []
    angles = []

    while 1:
        # Choose a random point in achievable space
        coordinates = [random.uniform(*limits) for limits in ROBOT_RANGE]
        try:
            # Account for joint3 introducing offset in get_inverse
            real_z = coordinates[2] - net_kinematics.ROBOT_L3
            theta0, theta1, theta2, _ = net_kinematics.get_inverse(coordinates[0], coordinates[1], real_z, -np.pi / 2)
            angles = [theta0, theta1, theta2]
            for i in range(3):
                if angles[i] < JOINT_LIMITS[i][0] or angles[i] > JOINT_LIMITS[i][1]:
                    raise ValueError
            three_joint_coordinates = [coordinates[0], coordinates[1], coordinates[2]]
            break

        except ValueError:
            # The randomly chosen coordinate was not reachable, try again
            pass

    return (three_joint_coordinates, angles)


def generate_position_samples(n_samples: int, save_file: Path) -> None:
    """
    Generate n_samples, scale if required, and save to pickle file
    """
    samples = []
    for i in range(n_samples):
        position = generate_position_constrained()
        positions = scale_values(position[0], ROBOT_RANGE, NNET_INPUT_RANGE)
        angles = scale_values(position[1], JOINT_LIMITS, NNET_OUTPUT_RANGE)
        samples.append([positions, angles])

    with open(save_file, 'wb') as f:
        pickle.dump(samples, f)


def generate_multiple_sample_files(n_files: int, samples_per_file: int, save_directory: str):
    for i in range(n_files):
        filename = f'positiondata_{samples_per_file}_{i}.dat'
        generate_position_samples(samples_per_file, Path(save_directory, filename))
    print(f'Saved {n_files} sample files of size {samples_per_file} to {save_directory}/')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise ValueError(f'Position generator takes 3 arguments: n_files, samples_per_file, save_directory')
    generate_multiple_sample_files(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
