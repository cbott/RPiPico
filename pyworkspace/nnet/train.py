# train.py
# functions for training nnet models
import torch
from torch.optim import Adam
import matplotlib.pyplot as plt
from tqdm import tqdm
from models import InvKin

B = 2048            # Batch size
device = 'cpu'      # Device
niter = 1000        # Number of training steps

# Make model, loss, and optimizer
model = InvKin()
model = model.to(device)
optimizer = Adam(model.parameters(), lr=1e-4)

losses = []
for iter in tqdm(range(niter)):
    # Sample random point in "roughly" reachable space: [-.25, .25]**3
    xs = torch.rand((B, 3)) * .25 - .125
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

# Save model
torch.save(model.state_dict(), 'chkpt.pth')

# Save loss plot
plt.plot(losses)
plt.savefig('losses.png')
plt.show()
