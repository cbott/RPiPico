# nnet.py
# Core logic for 3-DOF robot arm control neural network
import pickle
from pathlib import Path

import torch
from torch import nn
from torch.optim import sgd

DATA_FOLDER = 'trainingdata'


def load_sample_file(filename: str | Path) -> tuple[torch.Tensor, torch.Tensor]:
    """ returns (tensor of positions, tensor of angles) """
    with open(filename, 'rb') as f:
        samples = pickle.load(f)
    positions = torch.tensor([sample[0] for sample in samples]).float()
    angles = torch.tensor([sample[1] for sample in samples]).float()
    return (positions, angles)


class SimpleRobotNNet:
    n_inputs = 3  # Cartesian X/Y/Z coordinates
    n_outputs = 3  # Base, shoulder, and elbow joints
    n_hidden1 = 196
    n_hidden2 = 196
    learning_rate = 0.1

    def __init__(self, network: None | nn.Module = None):
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

    def save_state_to_pickle(self, save_file: Path | str) -> None:
        with open(save_file, 'wb') as f:
            pickle.dump(self.network, f)
        print(f'Saved nnet to file {save_file}')

    def train(self, files: list[Path] | list[str]) -> tuple[float, float]:
        """ Train 1 epoch

        files: list of file paths, pickle files with data to train on

        returns total training loss across all training data, number of samples trained on
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
        return total_loss, total_samples

    def test(self, files: list[Path] | list[str]) -> tuple[float, float]:
        """ Evaluate network

        files: list of file paths, pickle files with data to test on

        returns total testing loss across all samples, number of samples tested on
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
        return test_loss, test_samples

    def compute_joints(self, x: float, y: float, z: float) -> list[float]:
        """ Use nerual network to compute joint angles for a given target coordinate """
        inputs = torch.tensor([[x, y, z]]).float()
        self.network.eval()
        outputs = self.network(inputs)
        return outputs[0].tolist()
