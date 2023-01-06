# kinematics.py
# functions for performing kinematics operations
import torch


# See https://www.youtube.com/watch?v=rA9tm0gTln8 for DH parameter explanation
def get_dh_array(d: torch.Tensor, theta: torch.Tensor, r: torch.Tensor, alpha: torch.Tensor) -> torch.Tensor:
    """
    Construct a Denavit-Hartenberg transformation matrix in a way that
    ensures a Torch autodiff graph is still constructed

    d (0-d tensor):
        depth along previous joint's Z axis from the previous joint's origin to the common normal
        of the previous and next joint's Z axes
    theta (0-d tensor):
        angle, about previous joint's Z axis, between previous and next joints' X axes
    r (0-d tensor):
        distance from previous joint's origin to next joint's origin, projected on
        the previous joint's plane of rotation
    alpha (0-d tensor):
        angle, about next joint's X axis, between previous and next joints' Z axes
    """
    device = theta.device
    row_0 = torch.stack([torch.cos(theta), -torch.sin(theta) * torch.cos(alpha), torch.sin(theta) * torch.sin(alpha), r * torch.cos(theta)])
    row_1 = torch.stack([torch.sin(theta), torch.cos(theta) * torch.cos(alpha), -torch.cos(theta) * torch.sin(alpha), r * torch.sin(theta)])
    row_2 = torch.stack([torch.tensor(0.0).to(device), torch.sin(alpha), torch.cos(alpha), d])
    row_3 = torch.tensor([0., 0., 0., 1.]).to(device)
    return torch.stack([row_0, row_1, row_2, row_3])


def get_dh_array_vec(ds: torch.Tensor, thetas: torch.Tensor, rs: torch.Tensor, alphas: torch.Tensor) -> torch.Tensor:
    """
    Construct a batch of Denavit-Hartenberg matrices

    ds (torch.tensor) : shape (1, N)
        d parameters for each DH matrix in the returned vector
    thetas (torch.tensor) : shape (1, N)
        theta parameters for each DH matrix in the returned vector
    rs (torch.tensor) : shape (1, N)
        r parameters for each DH matrix in the returned vector
    alphas (torch.tensor) : shape (1, N)
        alpha parameters for each DH matrix in the returned vector
    """
    dhs = []
    for d, theta, r, alpha in zip(ds, thetas, rs, alphas):
        dh = get_dh_array(d, theta, r, alpha)
        dhs.append(dh)
    return torch.stack(dhs)
