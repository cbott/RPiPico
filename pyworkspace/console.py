# Serial console to interface with robot
import argparse
from serial import Serial


def translate(command: str) -> bytes:
    """
    Translate a human-readable command into machine serial instruction

    Commands are in the form "X I1 I2 ... In"
    Where X is a command character and I1..In are integer parameters separated by whitespace
    """
    instr = command[0].encode()
    params = command[1:].split()

    return instr + b"".join(i.to_bytes(2, "big") for i in map(int, params)) + b"\n"


def console(port: str, baudrate: int) -> None:
    """ Interactive serial console for robot control """
    with Serial(port, baudrate) as ser:
        print(f"Opened serial console with robot at {port}:{baudrate}")
        while 1:
            try:
                command = input("> ")
                command += "\n"
                ser.write(command.encode())
            except ValueError as e:
                print(f"Unable to translate command: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", type=str)
    parser.add_argument("baud", type=int)

    args = parser.parse_args()
    console(args.port, args.baud)
