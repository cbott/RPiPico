# models.py
# Pytorch nnet implementations
# Sections adapted from https://github.com/dangeng/inverse_kinematics
import torch
import torch.nn as nn
import numpy as np

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

        # Arm lengths (hardcoded!)
        self.rs = [.12, .115]

    def get_dh_array(self, d, theta, r, alpha):
        '''
        Ugly way to construct a dh matrix such that an
            autodiff graph is still constructed
            (idk if there's a better way to do this...)
        '''
        device = theta.device
        row_0 = torch.stack([torch.cos(theta), -torch.sin(theta) * torch.cos(alpha), torch.sin(theta) * torch.sin(alpha), r * torch.cos(theta)])
        row_1 = torch.stack([torch.sin(theta), torch.cos(theta) * torch.cos(alpha), -torch.cos(theta) * torch.sin(alpha), r * torch.sin(theta)])
        row_2 = torch.stack([torch.tensor(0.0).to(device), torch.sin(alpha), torch.cos(alpha), d])
        row_3 = torch.tensor([0., 0., 0., 1.]).to(device)
        dh = torch.stack([row_0, row_1, row_2, row_3])

        return dh

    def get_dh_array_vec(self, ds, thetas, rs, alphas):
        '''
        Ugly way to construct a batch of dh matrices
        '''
        dhs = []
        for d, theta, r, alpha in zip(ds, thetas, rs, alphas):
            dh = self.get_dh_array(d, theta, r, alpha)
            dhs.append(dh)

        dhs = torch.stack(dhs)
        return dhs

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

        dhs = self.get_dh_array_vec(zeros, thetas.T[0], zeros, ones * np.pi / 2)
        for theta, r in zip(thetas.T[1:], self.rs):
            new_dhs = self.get_dh_array_vec(zeros, theta, ones * r, zeros)
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
        thetas = self.net(x)
        pred_x, _ = self.forward_model(thetas)
        return thetas, pred_x
