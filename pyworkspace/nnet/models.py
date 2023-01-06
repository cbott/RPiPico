# models.py
# Pytorch nnet implementations
# Sections adapted from https://github.com/dangeng/inverse_kinematics
import numpy as np
import torch
import torch.nn as nn
from kinematics import get_dh_array_vec
from settings import *
from utilities import scale_values


class InvKin(nn.Module):
    def __init__(self, n_theta=3, hidden_dim=64):
        super().__init__()

        self.n_theta = n_theta

        # Inverse kinematics model: R^3 -> R^n
        self.net = nn.Sequential(
                        nn.Linear(3, hidden_dim),
                        nn.BatchNorm1d(hidden_dim),
                        nn.ReLU(),
                        nn.Linear(hidden_dim, hidden_dim),
                        nn.BatchNorm1d(hidden_dim),
                        nn.ReLU(),
                        nn.Linear(hidden_dim, hidden_dim),
                        nn.BatchNorm1d(hidden_dim),
                        nn.ReLU(),
                        nn.Linear(hidden_dim, n_theta)
                   )

        self.rs = [ROBOT_L1, ROBOT_L2]
        self.robot_range = torch.tensor(ROBOT_RANGE)
        self.net_input_range = torch.tensor(NNET_INPUT_RANGE)

    def forward_model(self, thetas):
        '''
        Runs forward kinematics model given thetas

        inputs
        ------
        thetas (torch.tensor) : shape (B, 3)
            batch of thetas for the arm

        returns
        -------
        x (torch.tensor) : shape (B, 3)
            x,y,z locations given thetas
        joint_locs (torch.tensor) : shape (B, 3, 3) (batch, joint_idx, xyz)
            location of joints for each arm. Last joint_idx
            corresponds to end effector (joints_locs[:, -1])
            and is equal to x
        '''
        B = thetas.shape[0]
        device = thetas.device

        # Make buffer of zeros and ones
        zeros = torch.zeros(thetas.shape[0]).to(device)
        ones = torch.ones(thetas.shape[0]).to(device)

        # Keep track of all joint locs
        joint_locs = [torch.zeros(thetas.shape[0], 3)]

        dhs = get_dh_array_vec(zeros, thetas.T[0], zeros, ones * np.pi / 2)
        for theta, r in zip(thetas.T[1:], self.rs):
            new_dhs = get_dh_array_vec(zeros, theta, ones * r, zeros)
            dhs = torch.bmm(dhs, new_dhs)

            # Append joint locs to list
            joint_locs.append(dhs[:, 0:3, 3])

        # joint_locs reshape: (b, 3) -> (b, 3 (joint idx), 3 (x,y,z))
        joint_locs = torch.stack(joint_locs, dim=1)

        # Return end effector position + all joint location
        x = dhs[:, 0:3, 3]
        return x, joint_locs

    def forward(self, x):
        '''
        Given xyz positions, runs the model to predict thetas for the
            robot arm to get to the xyz position. Also run a forward
            model on the thetas to get the real xyz position.

        inputs
        ------
        x (torch.tensor) : shape (B, 3)
            batch of xyz coordinates for inverse kinematics

        returns
        -------
        thetas (torch.tensor) : shape (B, 3)
            predicted inverse kinematics thetas
        pred_x (torch.tensor) : shape (B, 3)
            the actual xyz locations resulting from the
            predicted thetas
        '''
        scaled_x = scale_values(x, self.robot_range, self.net_input_range)
        thetas = self.net(scaled_x)
        pred_x, _ = self.forward_model(thetas)
        return thetas, pred_x
