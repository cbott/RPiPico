# micropython for Raspberyy Pi Pico
import binascii
from machine import UART, Pin
import struct
import utime
import random


class AX12:
    """ Hardware class for Dynamixel AX-12 servo line

    See https://emanual.robotis.com/docs/en/dxl/ax/ax-12a
    """
    MESSAGE_HEADER = 0xFF

    # Instructions
    CMD_PING          = 0X01
    CMD_READ          = 0X02
    CMD_WRITE         = 0X03
    CMD_REG_WRITE     = 0X04
    CMD_ACTION        = 0X05
    CMD_FACTORY_RESET = 0X06
    CMD_REBOOT        = 0X08
    CMD_SYNC_WRITE    = 0X83
    CMD_BULK_READ     = 0X92

    # Memory map
    #                   Address # Size # Access
    MEM_MODEL_NUMBER          = 0   # 2 R
    MEM_FIRMWARE_VERSION      = 2   # 1 R
    MEM_ID                    = 3   # 1 RW
    MEM_BAUD_RATE             = 4   # 1 RW
    MEM_RETURN_DELAY_TIME     = 5   # 1 RW
    MEM_CW_ANGLE_LIMIT        = 6   # 2 RW
    MEM_CCW_ANGLE_LIMIT       = 8   # 2 RW
    MEM_TEMPERATURE_LIMIT     = 11  # 1 RW
    MEM_MIN_VOLTAGE_LIMIT     = 12  # 1 RW
    MEM_MAX_VOLTAGE_LIMIT     = 13  # 1 RW
    MEM_MAX_TORQUE            = 14  # 2 RW
    MEM_STATUS_RETURN_LEVEL   = 16  # 1 RW
    MEM_ALARM_LED             = 17  # 1 RW
    MEM_SHUTDOWN              = 18  # 1 RW
    MEM_TORQUE_ENABLE         = 24  # 1 RW
    MEM_LED                   = 25  # 1 RW
    MEM_CW_COMPLIANCE_MARGIN  = 26  # 1 RW
    MEM_CCW_COMPLIANCE_MARGIN = 27  # 1 RW
    MEM_CW_COMPLIANCE_SLOPE   = 28  # 1 RW
    MEM_CCW_COMPLIANCE_SLOPE  = 29  # 1 RW
    MEM_GOAL_POSITION         = 30  # 2 RW
    MEM_MOVING_SPEED          = 32  # 2 RW
    MEM_TORQUE_LIMIT          = 34  # 2 RW
    MEM_PRESENT_POSITION      = 36  # 2 R
    MEM_PRESENT_SPEED         = 38  # 2 R
    MEM_PRESENT_LOAD          = 40  # 2 R
    MEM_PRESENT_VOLTAGE       = 42  # 1 R
    MEM_PRESENT_TEMPERATURE   = 43  # 1 R
    MEM_REGISTERED            = 44  # 1 R
    MEM_MOVING                = 46  # 1 R
    MEM_LOCK                  = 47  # 1 RW
    MEM_PUNCH                 = 48  # 2 RW

    # TODO: make a Broadcast() static method, might need to make send_packet more generic
    # TODO: should methods perform bounds checking/constrain inputs?

    def __init__(self, uart: UART, id: int):
        """
        uart: UART object the servo is connected to
        id: Hardware id of servo
        """
        self.uart = uart
        self.id = id

    def _send_uart(self, txdata: bytes):
        """ Send supplied data over UART """
        self.uart.write(txdata)

    def _send_instruction(self, instruction: int, *params: int):
        """ Send message to servo in Dynamixel format """
        # Dynamixel-defined Length field
        length = len(params) + 2

        checksum = (~(self.id + length + instruction + sum(params))) & 0xFF

        # TODO: make this more elegant
        message_contents = [AX12.MESSAGE_HEADER, AX12.MESSAGE_HEADER, self.id, length, instruction] + list(params) + [checksum]

        # Length of the message, in bytes
        num_bytes = len(message_contents)

        message = struct.pack('{}b'.format(num_bytes), *message_contents)
        self._send_uart(message)

    def write_led_status(self, status: bool):
        """
        status: desired LED status
        """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_LED, status)  # type: ignore

    def write_position(self, position: int):
        """
        position: 0-1023 target position to move to
        """
        # TODO: Write angle in degrees instead of 0-1023 position value?
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_GOAL_POSITION, position & 0xFF, position >> 8)

    # Note: set CW and CCW limit to 0 for "wheel" mode (no rotation limits)
    def write_cw_limit(self, position: int):
        """ Clockwise angle limit
        position: 0-1023 angle limit
        """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_CW_ANGLE_LIMIT, position & 0xFF, position >> 8)

    def write_ccw_limit(self, position: int):
        """ Counter-clockwise angle limit
        position: 0-1023 angle limit
        """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_CCW_ANGLE_LIMIT, position & 0xFF, position >> 8)

    def write_speed(self, speed: int):
        """ Moving Speed
        speed:
            If in Joint mode: 0-1023 corresponding to ~0-114 RPM
            If in Wheel mode: 0-2047 corresponding to 0-100% output power
        """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_MOVING_SPEED, speed & 0xFF, speed >> 8)

    def write_id(self, id: int):
        """ Change the network identifier of the servo

        id: the desired new ID for the servo [0-253]
        """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_ID, id)
        self.id = id

    def read_temperature(self):
        """ Present Tempearatrue

        Return temperature as read from the AX-12
        """
        # Send the read instruction
        data_length = 0x1  # Read 1 byte of data from the specified memory address
        self._send_instruction(AX12.CMD_READ, AX12.MEM_PRESENT_TEMPERATURE, data_length)

        # TODO: Clearing the input buffer breaks things?
        # uart0.read(uart0.any())  # Clear the input buffer

        # Set the TX pin to floating so it doesn't interfere with RX
        # TODO: Hard coded pin value
        txpin = Pin(0, mode=Pin.IN)

        # rxData = bytes()
        # while self.uart.any() > 0:
        #     rxData += self.uart.read(1)
        rxData = self.uart.read()  # Read in all available data

        if not rxData or len(rxData) < 7:
            raise ValueError('Did not receive response from AX12 device')

        # Response should be 7 bytes, discard all the beginning stuff
        # 0x FF  FF  03  03   00   33  C6
        #    HDR HDR ID  LEN  ERR  T   CHK
        response_data = rxData[-7:]
        r_id, r_len, r_err, r_temperature, r_checksum = struct.unpack('7B', response_data)
        checksum = (~(r_id + r_len + r_err + r_temperature)) & 0xFF

        # Set the UART back to send mode
        # TODO: instead of this just set the pin mode back
        uart0 = UART(0, baudrate=BAUDRATE, tx=txpin, rx=Pin(1))

        if checksum != r_checksum:
            raise ValueError('Calculated checksum {} does not match packet checksum {}'.format(checksum, r_checksum))

        return r_temperature

class Robot:
    NUM_JOINTS = 4
    JOINT_LIMITS = [
        (0, 1023),
        (200, 830),
        (45, 1000),
        (45, 1000)
    ]

    def __init__(self, uart: UART) -> None:
        self.joints = [AX12(uart, 0x1),
                       AX12(uart, 0x2),
                       AX12(uart, 0x3),
                       AX12(uart, 0x4)]

        for joint_num, joint in enumerate(self.joints):
            joint.write_cw_limit(self.JOINT_LIMITS[joint_num][0])
            joint.write_ccw_limit(self.JOINT_LIMITS[joint_num][1])
            joint.write_speed(100)

    def set_joint(self, joint_num: int, position: int) -> None:
        # TODO: Add bounds checking or read error flag after write
        # maybe write_position needs to return a bool and we pass that along?
        self.joints[joint_num].write_position(position)



BAUDRATE = 1000000

def test_servo():
    uart0 = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1))

    robot = Robot(uart0)

    while 1:
        num = int(input("Joint number (1-4): "))
        pos = int(input("Position (0-1023): "))
        robot.set_joint(num, pos)

test_servo()

