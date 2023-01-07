# utilities.py
# Helper functions for NNet
from typing import List, Tuple

import torch


class PositionSampler:
    """Class for generating groups of random numbers within the provided ranges"""
    def __init__(self, ranges: List[Tuple[float, float]]) -> None:
        """Initialize PositionSampler object.

        Args:
            ranges: List containing (low, high) values for each feature in the output
        """
        self.distributions = [torch.distributions.uniform.Uniform(*minmax) for minmax in ranges]

    def get_samples(self, num_samples: int) -> torch.Tensor:
        """Get a set of random points in the samper's range.

        Args:
            num_samples: number of rows in the returned tensor

        Returns:
            Tensor with size (N, M
            N: num_samples
            M: size of the ranges list the PositionSampler was initialized with
        """
        n = torch.Size([num_samples])
        return torch.stack([dist.sample(n) for dist in self.distributions]).T


def scale_values(values: torch.Tensor,
                 input_ranges: torch.Tensor,
                 output_ranges: torch.Tensor) -> torch.Tensor:
    """Map provided values from the input range to the output range using a linear scaling.

    Args:
        values : shape (N, M)
            2D tensor of values, N groups of M elements each which should be scaled according to the ranges
        input_ranges : shape (M, 2)
            tensor containing expected ranges for each of the M elements, minimum in column 0, maximum in column 1
        output_ranges : shape (M, 2)
            tensor containing desired ranges for each of the M elements, minimum in column 0, maximum in column 1

    Returns:
        Scaled tensor
    """
    return (values - input_ranges[:, 0]) \
        / (input_ranges[:, 1] - input_ranges[:, 0]) \
        * (output_ranges[:, 1] - output_ranges[:, 0]) \
        + output_ranges[:, 0]


# Legacy version
# def scale_values(values: Iterable[float],
#                  input_ranges: list[tuple[float, float]],
#                  output_ranges: list[tuple[float, float]]) -> list[float]:
#     """
#     Map provided values from the input range to the output range using a linear scaling

#     values: list of "real" values which have a range that falls within the corresponding element of input_ranges
#     input_ranges: list of tuples, each tuple representing the possible range of the corresponding coordinate
#     output_ranges: list of tuples, each tuple representing the range to scale that element to
#     """
#     result = []
#     for i, value in enumerate(values):
#         (input_min, input_max) = input_ranges[i]
#         (output_min, output_max) = output_ranges[i]
#         new_value = (value - input_min) / (input_max - input_min) * (output_max - output_min) + output_min
#         result.append(new_value)
#     return result
