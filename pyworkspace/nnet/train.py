# train.py
# functions for training nnet models
import argparse
from typing import List

import matplotlib.pyplot as plt
import torch
from models import InvKin
from torch.optim import Adam
from tqdm import tqdm

device = 'cpu'      # Device


def train_model(model: torch.nn.Module, batch_size: int, niter: int) -> List[float]:
    """ Train the provided model

    model: pytorch model to train
    batch_size: number of elements to train on per epoch
    niter: number of training epochs to run
    """
    # Make loss and optimizer
    model = model.to(device)
    optimizer = Adam(model.parameters(), lr=1e-4)

    losses = []
    for iter in tqdm(range(niter)):
        # Sample random point in "roughly" reachable space: [-.25, .25]**3
        xs = torch.rand((batch_size, 3)) * .25 - .125
        xs = xs.to(device)

        # Forward through model
        thetas, pred_x = model(xs)

        # Penalize L2 dist btwn target and actual position
        dist = torch.linalg.norm(xs - pred_x, dim=1)
        loss = dist.mean()

        # Optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # Track losses
        losses.append(loss.item())

    return losses


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output",
                        help="file to save resulting model to",
                        default="chkpt.pth")
    parser.add_argument("-b", "--batch", help="batch size", type=int, default=2048)
    parser.add_argument("-n", "--niter", help="number of training steps", type=int, default=1000)
    args = parser.parse_args()

    # Create and train a new model
    model = InvKin()
    losses = train_model(model, args.batch, args.niter)

    # Save model
    torch.save(model.state_dict(), args.output)

    # Save loss plot
    plt.plot(losses)
    plt.savefig('losses.png')
    plt.show()
