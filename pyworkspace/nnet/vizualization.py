# vizualization.py
# functions for plotting inverse kinematics results
import argparse

import matplotlib.pyplot as plt
import numpy as np
import torch
from models import InvKin
from settings import *
from utilities import PositionSampler


def plot_joint_locs(joint_locs: np.ndarray, target_locs: np.ndarray | None = None, save_path: str | None = None):
    """
    joint_locs (np array):
        joint locations of shape (batch, joint_idx, xyz) = (B, 3, 3)
    target_locs (np array):
        optional target locations to plot, of shape (batch, xyz)
    save_path:
        optional save path
    """
    # Make 3d plot
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_box_aspect(np.ptp(ROBOT_RANGE, axis=1))

    # Plot each arm
    for jls in joint_locs:
        ax.plot(jls[:, 0], jls[:, 1], jls[:, 2])

    # Plot each target
    if target_locs is not None:
        ax.scatter(target_locs[:, 0], target_locs[:, 1], target_locs[:, 2])

    # These ranges are hardcoded...
    ax.set_xlim(*ROBOT_RANGE[0])
    ax.set_ylim(*ROBOT_RANGE[1])
    ax.set_zlim(*ROBOT_RANGE[2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if save_path is not None:
        plt.savefig(save_path)
    else:
        plt.show()


def sanity_check(model, N=16, save_path=None):
    '''
    Spins all joints at constant rate, half revolution
    '''

    # Make thetas
    thetas = torch.zeros(N, 3)

    # Spin all joints by half revolution
    thetas[:, 0] = torch.linspace(0, 3.141592, N)
    thetas[:, 1] = torch.linspace(0, 3.141592, N)
    thetas[:, 2] = torch.linspace(0, 3.141592, N)

    # Get joint locs for each
    _, joint_locs = model.forward_model(thetas)
    joint_locs = joint_locs.numpy()

    plot_joint_locs(joint_locs, save_path=save_path)


def random_test(model: torch.nn.Module, N: int = 16, save_path: str | None = None):
    '''
    Sample random locations, and plot predictions
    '''
    with torch.no_grad():
        # Sample random point in "roughly" reachable space: [-.25, .25]**3
        sampler = PositionSampler(ROBOT_RANGE)
        xs = sampler.get_samples(N)

        # Forward through model
        thetas, pred_x = model(xs)

        # Get joint locs for each prediction
        _, joint_locs = model.forward_model(thetas)

    plot_joint_locs(joint_locs.numpy(), target_locs=xs.numpy(), save_path=save_path)


def error_heatmap(model: torch.nn.Module) -> None:
    """
    Create a 3D heatmap over the provided range, with color indicating
    the model's predition error at the coordinate
    """
    ranges = [
        [ROBOT_RANGE[0][0], ROBOT_RANGE[0][1], 18],  # X
        [ROBOT_RANGE[1][0], ROBOT_RANGE[1][1], 9],  # Y
        [ROBOT_RANGE[2][0], ROBOT_RANGE[2][1], 9]   # Z
    ]

    # Create 3D point cloud
    target_pts = []
    for x in np.linspace(*ranges[0]):
        for y in np.linspace(*ranges[1]):
            for z in np.linspace(*ranges[2]):
                target_pts.append(torch.tensor([x, y, z], dtype=torch.float))
    target_pts = torch.stack(target_pts)

    # Run the model on each point and get error
    with torch.no_grad():
        thetas, pred_pts = model(target_pts)
        dist = torch.linalg.norm(target_pts - pred_pts, dim=1).numpy()

    # creating figures
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect(np.ptp(ROBOT_RANGE, axis=1))

    img = ax.scatter(target_pts[:, 0], target_pts[:, 1], target_pts[:, 2], c=dist, cmap='gnuplot2', s=200)
    plt.colorbar(img)

    # adding title and labels
    ax.set_title("3D Position Error [mm]")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # displaying plot
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="filename of model to visualize")
    args = parser.parse_args()

    # Make model
    model = InvKin()

    # Load checkpoint
    chkpt = torch.load(args.model)
    model.load_state_dict(chkpt)
    model.eval()

    # Run visualization
    # sanity_check(model)
    random_test(model, 1)
    # error_heatmap(model)


if __name__ == "__main__":
    main()
