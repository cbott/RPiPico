import argparse
from collections import OrderedDict
import matplotlib.pyplot as plt
import os
from pathlib import Path
import pickle
import sys
import torch
from torch import nn
from torch.optim import sgd
from typing import List, Tuple

from kinematics import get_forward

N_INPUTS = 3
N_HIDDEN1 = 100
N_HIDDEN2 = 100
N_OUTPUTS = 3

TRAINING_EPOCHS = 1000
LEARNING_RATE = 0.01

DATA_FOLDER = 'constrained_samples'


def load_sample_file(filename) -> Tuple[torch.Tensor, torch.Tensor]:
    """ returns (List of positions, List of angles) """
    with open(Path(DATA_FOLDER, filename), 'rb') as f:  # type: ignore
        samples = pickle.load(f)
    positions = normalize_coordinates(torch.tensor([sample[0] for sample in samples]).float())
    angles = torch.tensor([sample[1] for sample in samples]).float()
    return (positions, angles)

def normalize_coordinates(t: torch.Tensor) -> torch.Tensor:
    """ Scale coordinates to improve nnet performance """
    return t / 200.0

def save_nnet_pickle(network, save_file):
    with open(save_file, 'wb') as f:  # type: ignore
        pickle.dump(network, f)
    print(f'Saved nnet to file {save_file}')


def train_network(network: nn.Module=None, save_file: str='nnet_save.dat'):
    if network is None:
        network = nn.Sequential(
            nn.Linear(N_INPUTS, N_HIDDEN1),
            nn.ReLU(),
            nn.Linear(N_HIDDEN1, N_HIDDEN2),
            nn.ReLU(),
            nn.Linear(N_HIDDEN2, N_OUTPUTS)
        )

    optimizer = sgd.SGD(network.parameters(), lr=LEARNING_RATE)
    loss_function = nn.MSELoss()

    # Load training/testing data
    all_files = os.listdir(DATA_FOLDER)
    testing_files = all_files[0:25]
    training_files = all_files[25:]

    results = [[], [], []]

    # Train
    for epoch in range(TRAINING_EPOCHS):
        # Set to training mode
        network.train()

        total_loss = 0
        total_samples = 0

        for training_file in training_files:
            target_position, true_angles = load_sample_file(training_file)

            # Make preditictions
            predictions = network(target_position)
            loss = loss_function(predictions, true_angles)

            # Back propogation
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # Note performance
            total_loss += loss.item() * true_angles.size(0)
            total_samples += true_angles.size(0)

        print(f'Epoch {epoch} | trained on {total_samples} samples | average loss: {total_loss / total_samples}')
        results[0].append(epoch)
        results[1].append(total_loss / total_samples)

        # Set to testing mode
        network.eval()
        test_loss = 0
        test_samples = 0
        with torch.no_grad():
            for testing_file in testing_files:
                target_position, true_angles = load_sample_file(testing_file)
                predictions = network(target_position)
                loss = loss_function(predictions, true_angles)

                test_loss += loss.item() * true_angles.size(0)
                test_samples += true_angles.size(0)

        print(f'Epoch {epoch} | tested on {test_samples} samples | average loss: {test_loss / test_samples}')
        results[2].append(test_loss / test_samples)


    save_nnet_pickle(network, save_file)

    plt.plot(results[0], results[1], results[0], results[2])
    plt.legend(['Training Loss', 'Testing Loss'])
    plt.show()


def test_nnet(network: nn.Module):
    network.eval()

    while 1:
        x = float(input('X: '))
        y = float(input('Y: '))
        z = float(input('Z: '))
        position = normalize_coordinates(torch.tensor([[x, y, z]]).float())
        (theta0, theta1, theta2) = network(position)[0].tolist()

        (real_x, real_y, real_z) = get_forward(theta0, theta1, theta2)
        dx = x - real_x
        dy = y - real_y
        dz = z - real_z

        error = (dx**2 + dy**2 + dz**2)**0.5

        print(f'Target position was ({x}, {y}, {z})')
        print(f'Nerual network recommends setting joint angles ({theta0}, {theta1}, {theta2})')
        print(f'These joint angles result in a position of ({real_x}, {real_y}, {real_z})')
        print(f'This is a cartesian error of {round(error, 4)} mm ({dx} in X, {dy} in Y, {dz} in Z)\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='If -i is selected, save resulting network to this file. If -e is selected, evaluate this file')
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-e', '--evaluate', help='Test a saved network against forward kinematics', action='store_true')
    group.add_argument('-i', '--input', help='File to save resulting PyTorch network to', type=str)
    args = parser.parse_args()

    if args.evaluate:
        with open(args.file, 'rb') as f:  # type: ignore
            net_load = pickle.load(f)
        test_nnet(net_load)

    else:
        net_load = None
        if args.input:
            with open(sys.argv[1], 'rb') as f:  # type: ignore
                net_load = pickle.load(f)

        train_network(net_load, sys.argv[1])
