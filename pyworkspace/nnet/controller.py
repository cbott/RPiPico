# controller.py
# Script for generating and sending position commands to robot over serial
# import argparse
import os
from pathlib import Path
import pickle
# from serial import Serial
import time
import serial
import matplotlib.pyplot as plt
import position_generator
import net_kinematics
from nnet import SimpleRobotNNet
from math import pi

BAUD_RATE = 115200

TRAINING_FOLDER = 'trainingdata'
TESTING_FOLDER = 'testingdata'

EVALUATION_EPOCHS = [1, 5, 10, 50, 100, 500, 1000]

TARGET_PATH = [
    (75, 90, 20),
    (75, 175, 20),
    (-75, 175, 20),
    (-75, 90, 20)
]


def run_evaluation(model: SimpleRobotNNet, ser):
    for position in TARGET_PATH:
        target = position_generator.scale_values(position, position_generator.OUTPUT_SPACE, position_generator.SCALED_RANGE)
        thetas = model.compute_joints(*target)
        thetas = position_generator.scale_values(thetas, position_generator.SCALED_RANGE, position_generator.JOINT_LIMITS)

        (real_x, real_y, real_z) = net_kinematics.get_forward(thetas[0], thetas[1], thetas[2])
        dx = position[0] - real_x
        dy = position[1] - real_y
        dz = position[2] - real_z

        error = (dx**2 + dy**2 + dz**2)**0.5

        print(f'Target position was {position}')
        print(f'Nerual network recommends setting joint angles {thetas}')
        print(f'These joint angles result in a position of ({real_x}, {real_y}, {real_z})')
        print(f'This is a cartesian error of {round(error, 4)} mm ({dx} in X, {dy} in Y, {dz} in Z)\n')

        theta3 = -pi / 2.0 - thetas[1] - thetas[2]
        if theta3 < -pi:
            theta3 += 2*pi
        if theta3 > pi:
            theta3 -= 2*pi

        ser.write(f'A {round(thetas[0], 5)} {round(thetas[1], 5)} {round(thetas[2], 5)} {round(theta3, 5)}\n'.encode())
        time.sleep(1.5)
        ser.write(b'H\n')
        time.sleep(1.5)


def main():
    """
    Train and run neural network for robot control
    """
    save_file = "results/nnet_1e4epoch_scaled.pkl"

    with open(save_file, 'rb') as f:
        network = pickle.load(f)

    robot = SimpleRobotNNet(network)
    # robot = SimpleRobotNNet()
    # ser = Serial('/dev/ttyACM0', 115200)

    training_files = [Path(TRAINING_FOLDER, f) for f in os.listdir(TRAINING_FOLDER)]
    testing_files = [Path(TESTING_FOLDER, f) for f in os.listdir(TESTING_FOLDER)]
    results = [[], [], []]

    # Train for N epochs
    print("Beginning Training")
    for epoch in range(10000):
        # print(f'Training Epoch {epoch}')
        train_loss, train_samples = robot.train(training_files)
        test_loss, test_samples = robot.test(testing_files)

        avg_train = train_loss / train_samples
        avg_test = test_loss / test_samples

        results[0].append(epoch)
        results[1].append(avg_train)
        results[2].append(avg_test)

        # if epoch in EVALUATION_EPOCHS:
        print(f'Epoch {epoch} completed with Testing Loss {avg_test}, Training Loss {avg_train}')
            # run_evaluation(robot, ser)


    with open('epoch_results.csv', 'w') as f:
        f.write(f'Training Loss,Testing Loss\n')
        for i in range(len(results[1])):
            f.write(f'{results[1][i]},{results[2][i]}\n')

    robot.save_state_to_pickle('results/nnet_save.pkl')
    # ser.close()

    plt.plot(results[0], results[1], results[0], results[2])
    plt.legend(['Training Loss', 'Testing Loss'])
    plt.xlabel('Training Epoch')
    plt.ylabel('Loss')
    plt.show()


def test_nnet():
    save_file = "results/nnet_1e4epoch_scaled.pkl"

    with open(save_file, 'rb') as f:
        network = pickle.load(f)

    robot = SimpleRobotNNet(network)

    while 1:
        x = float(input('X: '))
        y = float(input('Y: '))
        z = float(input('Z: '))
        target = position_generator.scale_values([x, y, z], position_generator.OUTPUT_SPACE, position_generator.SCALED_RANGE)
        thetas = robot.compute_joints(*target)
        thetas = position_generator.scale_values(thetas, position_generator.SCALED_RANGE, position_generator.JOINT_LIMITS)

        (real_x, real_y, real_z) = net_kinematics.get_forward(thetas[0], thetas[1], thetas[2])
        dx = x - real_x
        dy = y - real_y
        dz = z - real_z

        error = (dx**2 + dy**2 + dz**2)**0.5

        print(f'Target position was ({x}, {y}, {z})')
        print(f'Nerual network recommends setting joint angles {thetas}')
        print(f'These joint angles result in a position of ({real_x}, {real_y}, {real_z})')
        print(f'This is a cartesian error of {round(error, 4)} mm ({dx} in X, {dy} in Y, {dz} in Z)\n')


# TODO: Make a real and fake robot interface, one which prints results, one which is the real thing
# Maybe a third for like, training only that just prints loss out
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
    # test_nnet()
