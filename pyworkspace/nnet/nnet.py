
# nnet.py
# script for training and testing neural networks for 3-DOF robot arm
import argparse
import matplotlib.pyplot as plt
import os
from pathlib import Path
import pickle
import torch
from torch import nn
from torch.optim import sgd
from typing import List, Tuple, Optional

from net_kinematics import get_forward

DATA_FOLDER = 'trainingdata'


def load_sample_file(filename: str) -> Tuple[torch.Tensor, torch.Tensor]:
    """ returns (List of positions, List of angles) """
    with open(Path(DATA_FOLDER, filename), 'rb') as f:
        samples = pickle.load(f)
    positions = normalize_coordinates(torch.tensor([sample[0] for sample in samples]).float())
    angles = torch.tensor([sample[1] for sample in samples]).float()
    return (positions, angles)


class SimpleRobotNNet:
    n_inputs = 3  # Cartesian X/Y/Z coordinates
    n_outputs = 3  # Base, shoulder, and elbow joints
    n_hidden1 = 196
    n_hidden2 = 196
    learning_rate = 0.1

    def __init__(self, network: Optional[nn.Module] = None):
        """ Initialize the SimpleRobotNNet

        network: if provided, pytorch nn module to be used
        """
        # TODO: should make n_hidden configurable here or at least parse the provided network to figure out what was provided
        if network is None:
            self.network = nn.Sequential(
                nn.Linear(self.n_inputs, self.n_hidden1),
                nn.ReLU(),
                nn.Linear(self.n_hidden1, self.n_hidden2),
                nn.ReLU(),
                nn.Linear(self.n_hidden2, self.n_outputs)
            )
        else:
            self.network = network

        # This could all be made configurable as well, or set each training round?
        self.optimizer = sgd.SGD(self.network.parameters(), lr=self.learning_rate)
        self.loss_function = nn.MSELoss()

    def save_state_to_pickle(self, save_file: str) -> None:
        with open(save_file, 'wb') as f:
            pickle.dump(self.network, f)
        print(f'Saved nnet to file {save_file}')

    def train(self, files: List[str]) -> float:
        """ Train 1 epoch

        files: list of file paths, pickle files with data to train on

        returns total training loss across all training data
        """
        # Set to training mode
        self.network.train()

        total_loss = 0
        total_samples = 0

        # TODO: Need to evaluate if it actually makes sense to split into multiple files
        # or if we should just accept a single big file and train on that
        # Also need to evaluate taking the data directly to avoid repead unpickle operations
        for training_file in files:
            target_position, true_angles = load_sample_file(training_file)

            # Make preditictions
            predictions = self.network(target_position)
            loss = self.loss_function(predictions, true_angles)

            # Back propogation
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            # Note performance
            total_loss += loss.item() * true_angles.size(0)
            total_samples += true_angles.size(0)

        # print(f'trained on {total_samples} samples | average loss: {total_loss / total_samples}')
        return total_loss

    def test(self, files: List[str]) -> float:
        """ Evaluate network

        files: list of file paths, pickle files with data to test on

        returns total testing loss across all samples
        """
        # Set to testing mode
        self.network.eval()
        test_loss = 0
        test_samples = 0
        with torch.no_grad():
            for testing_file in files:
                target_position, true_angles = load_sample_file(testing_file)
                predictions = self.network(target_position)
                loss = self.loss_function(predictions, true_angles)

                test_loss += loss.item() * true_angles.size(0)
                test_samples += true_angles.size(0)

        # print(f'tested on {test_samples} samples | average loss: {test_loss / test_samples}')
        return test_loss


# NNet constants
N_INPUTS = 3
N_HIDDEN1 = 196
N_HIDDEN2 = 196
N_OUTPUTS = 3

TRAINING_EPOCHS = 100
LEARNING_RATE = 0.1


def normalize_coordinates(t: torch.Tensor) -> torch.Tensor:
    """ Scale coordinates to improve nnet performance """
    return t / 200.0

def save_nnet_pickle(network: nn.Module, save_file: str) -> None:
    with open(save_file, 'wb') as f:
        pickle.dump(network, f)
    print(f'Saved nnet to file {save_file}')

def create_network() -> nn.Module:
    return nn.Sequential(
        nn.Linear(N_INPUTS, N_HIDDEN1),
        nn.ReLU(),
        nn.Linear(N_HIDDEN1, N_HIDDEN2),
        nn.ReLU(),
        nn.Linear(N_HIDDEN2, N_OUTPUTS)
    )

def train_network(network: Optional[nn.Module]=None, save_file: str='nnet_save.dat'):
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

    # TODO: Add axis labels
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
    parser = argparse.ArgumentParser(description='Train a neural network for robot control')
    # TODO: this is bad, make better arguments
    parser.add_argument('file', help='If -i is selected, save resulting network to this file. If -e is selected, evaluate this file')
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-e', '--evaluate', help='Test a saved network against forward kinematics', action='store_true')
    group.add_argument('-i', '--input', help='File containing existing network to train', type=str)
    args = parser.parse_args()

    if args.evaluate:
        with open(args.file, 'rb') as f:
            net_load = pickle.load(f)
        test_nnet(net_load)

    else:
        net_load = None
        if args.input:
            with open(args.file, 'rb') as f:
                net_load = pickle.load(f)

        train_network(net_load, args.file)

# TODO
# should have this take trainingdata and testingdata folder paths, open each of those separately
# net evaluation should move to a different file, more similar to controller than training
# position generation remains as a separate file

# Make nnet into a class that can be used from controller.py
# Inputs and outputs should be scaled, probably (-1, 1) but leave flexible to test this scale and see about performance of the net
# Probably want to set the position generator to do this for us so we don't re-compute every epoch