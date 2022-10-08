# controller.py
# Script for generating and sending position commands to robot over serial
import argparse
import os
from pathlib import Path

import serial
import matplotlib.pyplot as plt

from nnet.nnet import SimpleRobotNNet

BAUD_RATE = 115200

TRAINING_FOLDER = 'trainingdata'
TESTING_FOLDER = 'testingdata'

def main():
    """
    Not quite sure what this will do yet, starting out with just running the nnet
    """
    robot = SimpleRobotNNet()

    training_files = [Path(TRAINING_FOLDER, f) for f in os.listdir(TRAINING_FOLDER)]
    testing_files = [Path(TESTING_FOLDER, f) for f in os.listdir(TESTING_FOLDER)]
    results = [[], [], []]

    # Train for 100 epochs
    for i in range(100):
        train_loss, train_samples = robot.train(training_files)
        test_loss, test_samples = robot.test(testing_files)

        results[0].append(i)
        results[1].append(train_loss / train_samples)
        results[2].append(test_loss / test_samples)

    robot.save_state_to_pickle('results/nnet_save.pkl')

    # TODO: Add axis labels
    plt.plot(results[0], results[1], results[0], results[2])
    plt.legend(['Training Loss', 'Testing Loss'])
    plt.show()

    # TODO: Need a way to unscale nnet outputs back into real world units
    # this doesn't really belong in the robot interface class
    # should just have some helper functions to do this probably

class RobotInterface:
    def __init__(self, port):
        self._port = port
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = BAUD_RATE

    def __enter__(self):
        self._serial.open()
        return self

    def __exit__(self):
        self._serial.close()

    def set_values(self, j1: int, j2: int, j3: int) -> bytes:
        """ Set robot joint values """
        message = b's'  # command s = set values
        for val in (j1, j2, j3):
            # Send values as 2 bytes
            message += val.to_bytes(2, byteorder='big')
        self._serial.write(message)
        return message


if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('port', type=str, help='Serial port for robot communication')
    # args = parser.parse_args()

    # with RobotInterface(args.port) as robot:
    #     while 1:
    #         j1 = int(input('J1: '))
    #         j2 = int(input('J2: '))
    #         j3 = int(input('J3: '))
    #         print(robot.set_values(j1, j2, j3))
    main()
