# train.py
# functions for training nnet models
import argparse
from typing import Any, List, Tuple

import matplotlib.pyplot as plt
import torch
from models import InvKin
from settings import *
from torch.optim import SGD, Adam
from tqdm import tqdm
from utilities import PositionSampler

device = 'cpu'      # Device


def get_average_error_for_key_points(model: torch.nn.Module):
    target_pts = torch.tensor([
        [-100.0, 0, 0],
        [-100, 100, 50],
        [0, 150, 100],
        [100, 100, 50],
        [100, 0, 0],
    ])

    # Run the model on each point and get error
    with torch.no_grad():
        thetas, pred_pts = model(target_pts)
        av_dist = torch.linalg.norm(target_pts - pred_pts, dim=1).mean().item()

    return av_dist


def evaluate_parameters():
    n_training_points = 2048 * 1000
    save_file = "results/parameter_eval.csv"

    test_cases = [
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 2048, "lr": 1e-4, "batch_norm": False},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 2048, "lr": 1e-4, "batch_norm": True},
        {"optimizer": SGD, "hidden": 3, "nodes": 64, "batch": 2048, "lr": 1e-4, "batch_norm": True},
        {"optimizer": Adam, "hidden": 2, "nodes": 96, "batch": 2048, "lr": 1e-4, "batch_norm": True},
        {"optimizer": Adam, "hidden": 3, "nodes": 96, "batch": 2048, "lr": 1e-4, "batch_norm": True},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 1024, "lr": 1e-4, "batch_norm": True},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 256, "lr": 1e-4, "batch_norm": True},
        {"optimizer": SGD, "hidden": 3, "nodes": 64, "batch": 1024, "lr": 1e-4, "batch_norm": True},
        {"optimizer": SGD, "hidden": 3, "nodes": 64, "batch": 256, "lr": 1e-4, "batch_norm": True},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 2048, "lr": 1e-3, "batch_norm": True},
        {"optimizer": SGD, "hidden": 3, "nodes": 64, "batch": 2048, "lr": 1e-3, "batch_norm": True},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 256, "lr": 1e-3, "batch_norm": True},
        {"optimizer": SGD, "hidden": 3, "nodes": 64, "batch": 1024, "lr": 1e-4, "batch_norm": False},
        {"optimizer": Adam, "hidden": 3, "nodes": 64, "batch": 256, "lr": 1e-4, "batch_norm": False},
        {"optimizer": SGD, "hidden": 3, "nodes": 96, "batch": 2048, "lr": 1e-3, "batch_norm": True},
    ]

    for num, case in enumerate(test_cases):
        model = InvKin(n_hidden=case["hidden"], hidden_dim=case["nodes"])

        epochs = int(n_training_points / case["batch"])
        losses, runtime = train_model(model,
                                      batch_size=case["batch"],
                                      niter=epochs,
                                      optimizer_class=case["optimizer"],
                                      learning_rate=case["lr"])
        final_loss = losses[-1]

        model.eval()
        av_dist = get_average_error_for_key_points(model)

        # Save model
        torch.save(model.state_dict(), f"results/parameter_eval_model_{num:02}.pth")
        # Save loss plot
        plt.figure()
        plt.plot(losses)
        plt.savefig(f"results/parameter_eval_losses_{num:02}.png")
        # Write results to CSV
        with open(save_file, "a") as csvfile:
            params_to_save = list(case.values()) + [n_training_points, final_loss, av_dist, runtime]
            csvfile.write(",".join(map(str, params_to_save)))
            csvfile.write("\n")


def train_model(model: torch.nn.Module,
                batch_size: int,
                niter: int,
                optimizer_class: Any,
                learning_rate: float = 1e-4) -> Tuple[List[float], float]:
    """ Train the provided model

    model: pytorch model to train
    batch_size: number of elements to train on per epoch
    niter: number of training epochs to run

    returns: losses for each training epoch
    """
    # Make loss and optimizer
    model = model.to(device)
    optimizer = optimizer_class(model.parameters(), lr=learning_rate)

    sampler = PositionSampler(ROBOT_RANGE)

    losses = []
    with tqdm(range(niter)) as iter:
        for _ in iter:
            # Sample random point in reachable space:
            target_coords = sampler.get_samples(batch_size)
            target_coords = target_coords.to(device)

            # Forward through model
            thetas, pred_coords = model(target_coords)

            # Penalize L2 dist btwn target and actual position
            dist = torch.linalg.norm(target_coords - pred_coords, dim=1)
            loss = dist.mean()

            # Optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # Track losses
            losses.append(loss.item())
        elapsed = iter.format_dict["elapsed"]

    return losses, elapsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", help="file to save resulting model to", default="chkpt.pth")
    parser.add_argument("-b", "--batch", help="batch size", type=int, default=2048)
    parser.add_argument("-n", "--niter", help="number of training steps", type=int, default=1000)
    args = parser.parse_args()

    # Create and train a new model
    model = InvKin()
    losses = train_model(model, args.batch, args.niter, Adam)

    # Save model
    torch.save(model.state_dict(), args.output)

    # Save loss plot
    plt.plot(losses)
    plt.savefig("losses.png")
    plt.show()


if __name__ == "__main__":
    # main()
    evaluate_parameters()
