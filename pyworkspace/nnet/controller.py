# controller.py
# Script for generating and sending position commands to robot over serial
import argparse
import serial

BAUD_RATE = 115200


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
    parser = argparse.ArgumentParser()
    parser.add_argument('port', type=str, help='Serial port for robot communication')
    args = parser.parse_args()

    with RobotInterface(args.port) as robot:
        while 1:
            j1 = int(input('J1: '))
            j2 = int(input('J2: '))
            j3 = int(input('J3: '))
            print(robot.set_values(j1, j2, j3))
