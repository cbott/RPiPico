# micropython for Raspberyy Pi Pico
from collections import namedtuple
from machine import UART, Pin
from math import atan2, cos, pi, radians, sin
import struct
import utime


class AX12ResponseError(Exception):
    pass  # TODO: Subclass further?

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
    # TODO: Rewrite so that all memory values have getters and setters? Can access as x = goal_position or goal_position = x

    def __init__(self, uart: UART, id: int, rxpin: Pin):
        """
        uart: UART object the servo is connected to
        id: Hardware id of servo
        rxpin: Pin object controlling the connected tri-state buffer. 0=tx, 1=rx
        """
        self.uart = uart
        self.id = id
        self.rxpin = rxpin

    def _send_uart(self, txdata: bytes):
        """ Send supplied data over UART """
        self.uart.write(txdata)

    def _send_instruction(self, instruction: int, *params: int):
        """ Send message to servo in Dynamixel format, return list of response parameters """
        # Set to TX mode in advance of writing to the bus
        self.rxpin.value(0)  # TODO: don't hard code 0/1?

        # Dynamixel-defined Length field
        length = len(params) + 2

        checksum = (~(self.id + length + instruction + sum(params))) & 0xFF

        # TODO: make this more elegant
        message_contents = [AX12.MESSAGE_HEADER, AX12.MESSAGE_HEADER, self.id, length, instruction] + list(params) + [checksum]

        # Length of the message, in bytes
        num_bytes = len(message_contents)

        message = struct.pack('{}b'.format(num_bytes), *message_contents)

        # Send the instruction
        # print('[0x{:x}] Sending message {}'.format(self.id, message))
        self._send_uart(message)
        utime.sleep_us(1)  # Empirically determined delay to let uart finish transmitting I guess?

        # Read the status packet response
        self.rxpin.value(1)
        utime.sleep_ms(10)  # Dynamixel response delay is 500us, plus a bit of margin TODO: re-evaluate. 5ms didn't seem to be reliable with multiple servos on the bus
        rxData = self.uart.read()  # Read in all available data
        self.rxpin.value(0)

        if not rxData or len(rxData) < 6:
            print(rxData)
            raise AX12ResponseError('[0x{:x}] Did not receive response from AX12 device'.format(self.id))

        # Response should be 7 bytes
        # 0x FF  FF  03  03   00   [XX] [XX] [...]  C6
        #    HDR HDR ID  LEN  ERR  [P1] [P2] [P...] CHK
        # For some reason we also seem to read in a \x00 first, throw that out if it exists
        if rxData.startswith(b'\x00'):
            rxData = rxData[1:]
        if not rxData.startswith(bytes(2 * [AX12.MESSAGE_HEADER])):
            raise AX12ResponseError('[0x{:x}] Recieved data {} does not have expected message header'.format(self.id, rxData))

        # print('Data', rxData)
        # response = struct.unpack('7B', rxData)
        r_id = rxData[2]
        r_len = rxData[3]
        r_err = rxData[4]
        r_params = [i for i in rxData[5:-1]]
        r_checksum = rxData[-1]

        # print('Decoded', r_id, r_len, r_err, r_params, r_checksum)
        checksum = (~(r_id + r_len + r_err + sum(r_params))) & 0xFF

        if checksum != r_checksum:
            raise AX12ResponseError('[0x{:x}] Calculated checksum {} does not match packet checksum {}'.format(self.id, checksum, r_checksum))

        # Error codes
        # 1000000 Instruction Error   In case of sending an undefined instruction or delivering the action instruction without the Reg Write instruction, it is set as 1
        # 0100000 Overload Error      When the current load cannot be controlled by the set Torque, it is set as 1
        # 0010000 Checksum Error      When the Checksum of the transmitted Instruction Packet is incorrect, it is set as 1
        # 0001000 Range Error         When an instruction is out of the range for use, it is set as 1
        # 0000100 Overheating Error   When internal temperature of DYNAMIXEL is out of the range of operating temperature set in the Control table, it is set as 1
        # 0000010 Angle Limit Error   When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit , it is set as 1
        # 0000001 Input Voltage Error When the applied voltage is out of the range of operating voltage set in the Control table, it is as 1
        if r_err:
            raise AX12ResponseError('[0x{:x}] Status packet returned nonzero error byte {:08b}'.format(self.id, r_err))

        num_parameters = r_len - 2  # Dynamixel spec
        if num_parameters != len(r_params):
            raise AX12ResponseError('[0x{:x}] Unexpected number of parameters received. Expected {} parameter(s), received [{}]'.format(self.id, num_parameters, r_params))

        return r_params

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

    def write_torque_enable(self, enable: bool):
        """ Enable or disable servo torque output """
        self._send_instruction(AX12.CMD_WRITE, AX12.MEM_TORQUE_ENABLE, enable)  # type: ignore

    def read_temperature(self) -> int:
        """ Present Tempearatrue

        Return temperature as read from the AX-12
        """
        # Send the read instruction
        data_length = 0x1  # Read 1 byte of data from the specified memory address
        r_params = self._send_instruction(AX12.CMD_READ, AX12.MEM_PRESENT_TEMPERATURE, data_length)
        if len(r_params) != 1:
            raise ValueError('Received {} response parameters, expected 1'.format(len(r_params)))
        return r_params[0]

    def read_position(self) -> int:
        """ Present Position

        Return AX-12 position in counts (0-1023)
        """
        r_params = self._send_instruction(AX12.CMD_READ, AX12.MEM_PRESENT_POSITION, 0x2)
        return r_params[0] + (r_params[1] << 8)

# TODO: maybe this is class specific, store in Robot
Pose = namedtuple('Pose', ['theta0', 'theta1', 'theta2', 'theta3'])


# TODO: Use consistent nomenclature
# angle = radians, joint angle in robot coordinate frame
# coordinate/position? = xyz coordinates
# value = raw servo 0-1023 position

class Robot:
    NUM_JOINTS = 4  # TODO: seems a bit silly to have this
    JOINT_LIMITS = [
        (0, 1023),
        (200, 830),
        (45, 1000),
        (45, 1000)
    ]
    # Beam lengths [mm]
    L1 = 120
    L2 = 115
    L3 = 58

    def __init__(self, servo1: AX12, servo2: AX12, servo3: AX12, servo4: AX12) -> None:
        self.joints = [servo1, servo2, servo3, servo4]

        for joint_num, joint in enumerate(self.joints):
            joint.write_cw_limit(self.JOINT_LIMITS[joint_num][0])
            joint.write_ccw_limit(self.JOINT_LIMITS[joint_num][1])
            joint.write_speed(100)

        self.prev_angles = Pose(pi/2, 2, -pi/2, -pi/2)
        self.set_joint_angles(self.prev_angles)

    def set_joint_value(self, joint_num: int, position: int) -> None:
        """ Set joint position directly as Servo value, after rounding to nearest int """
        # TODO: Add bounds checking or read error flag after write
        # maybe write_position needs to return a bool and we pass that along?
        self.joints[joint_num].write_position(position)
        # TODO: find a way to cleanly merge with next method, maybe passing None for a list value skips it?

    def set_joint_values(self, values: list) -> None:
        # TODO: set speeds here too? Probably not but then maybe a standalone speed setting method?
        for i in range(len(self.joints)):
            self.set_joint_value(i, values[i])

    def set_joint_angles(self, angles: Pose, speed: int = 100) -> bool:
        """ Set angles in radians """
        # Determine motion speeds
        # TODO: should we make a standard "angle list" type to pass around? Probably not but could still hold in a list
        n = len(angles)  # TODO: should use class constant num joints
        deltas = [abs(angles[i] - self.prev_angles[i]) for i in range(n)]
        max_delta = max(deltas)
        if max_delta > 0:
            for joint_num in range(n):
                self.joints[joint_num].write_speed(int(speed * deltas[joint_num] / max_delta))
        self.prev_angles = angles

        # Convert angle to servo counts
        # TODO: determine if 0.29 is accurate
        COUNTS_PER_RADIAN = 197.571654  # 0.29 degrees per count
        values = [
            (angles.theta0 * COUNTS_PER_RADIAN) + 201,
             822 - (angles.theta1 * COUNTS_PER_RADIAN),  # Or (angles.theta1 * COUNTS_PER_RADIAN) + 201 if we fix servo direction
             512 - (angles.theta2 * COUNTS_PER_RADIAN),  # Or (angles.theta2 * COUNTS_PER_RADIAN) + 512
             512 - (angles.theta3 * COUNTS_PER_RADIAN)  # Or (angles.theta3 * COUNTS_PER_RADIAN) + 512
        ]
        values = [int(round(p, 0)) for p in values]

        for joint in range(self.NUM_JOINTS):
            limits = self.JOINT_LIMITS[joint]
            if not limits[0] <= values[joint] <= limits[1]:
                return False

        self.set_joint_values(values)
        return True

    @classmethod
    def inverse_kinematics(cls, x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False) -> Pose:
        """ Determine necessary joint angles to reach a position """
        # Solve for joint 0 angle
        theta0 = atan2(y, x)
        if invert_base:
            # When we want to "reach over backwards" the base points the opposite direction of the desired point
            theta0 += pi
        # Constrain base to the valid servo range of -pi/3 to 8pi/6
        # We may still end up in an invalid range, caller must ensure this is not the case
        if theta0 < -pi/3:
            theta0 += 2*pi
        if theta0 > 8*pi/6:
            theta0 -= 2*pi

        # Translate end effector coordinates to joint 3 coordinates
        d_z = cls.L3 * sin(end_angle)
        d_r = cls.L3 * cos(end_angle)
        x = x - d_r * cos(theta0)
        y = y - d_r * sin(theta0)
        z = z - d_z

        # Solve for joint 2 angle
        dist_squared = x**2 + y**2 + z**2
        # solve for cos(theta2) using law of cosines
        cos_theta2 = (dist_squared - cls.L1**2 - cls.L2**2) / (2 * cls.L1 * cls.L2)
        if not -1 <= cos_theta2 <= 1:
            raise ValueError("Target position {} not achievable".format((x, y, z)))
        sin_theta2 = -(1 - cos_theta2**2)**0.5
        if invert_elbow:
            sin_theta2 *= -1
        theta2 = atan2(sin_theta2, cos_theta2)

        # Solve for joint 1 angle
        r = (x**2 + y**2)**0.5
        alpha = atan2(z, r)
        beta = atan2(cls.L2 * sin(theta2), cls.L2 * cos(theta2) + cls.L1)
        theta1 = alpha - beta

        # Make corrections for flipped base
        if invert_base:
            # Base has been set to face away from the target point, reflect remaining joints about z-r plane
            theta1 = pi - theta1
            theta2 *= -1

        # Solve for joint 3 angle
        # TODO: make this play nice with inverted base
        theta3 = end_angle - theta1 - theta2
        # TODO: do this with mod math
        if theta3 < -pi:
            theta3 += 2*pi
        if theta3 > pi:
            theta3 -= 2*pi

        return Pose(theta0, theta1, theta2, theta3)

    @classmethod
    def numerical_ik_solver(cls):
        pass

    def disable_torque(self) -> None:
        for joint in self.joints:
            joint.write_torque_enable(False)

    def get_joint_values(self) -> list:
        return [joint.read_position() for joint in self.joints]

    def set_position(self, x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False, speed: int = 100) -> bool:
        """ Move the end effector to the specified cartesian coordinates

        end_angle: desired angle of the end effector relative to horizontal, in radians
        invert_base: set to True to "reach over backward" to hit the specified location
        invert_elbow: set to True to reach the specified locaton with inverted elbow geometry
        speed: maximum joint speed, servo units

        returns: True if position is set, False if position is unachievable
        """
        angles = self.inverse_kinematics(x, y, z, end_angle, invert_base, invert_elbow)
        print('Setting joint angles of {}, {}, {}, {} degrees'.format(angles.theta0*180/pi, angles.theta1*180/pi, angles.theta2*180/pi, angles.theta3*180/pi))
        return self.set_joint_angles(angles, speed)


BAUDRATE = 1000000

def test_servo():
    uart0 = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1))

    directionpin = Pin(2, Pin.OUT)
    directionpin.value(0)
    servo1 = AX12(uart0, 0x2, directionpin)

    # servo1.write_position(512)
    servo1.write_cw_limit(0)
    servo1.write_torque_enable(False)

    for i in range(1000):
        response = servo1.read_position()
        print('Position: ', response)
        utime.sleep(0.5)


def test_robot():
    uart0 = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1))


    directionpin = Pin(2, Pin.OUT)
    directionpin.value(0)

    robot = Robot(AX12(uart0, 0x1, directionpin),
                  AX12(uart0, 0x2, directionpin),
                  AX12(uart0, 0x3, directionpin),
                  AX12(uart0, 0x4, directionpin))

    while 1:
        coords = input("Coordinates: ")
        coords = [float(c) for c in coords.split()]
        if len(coords) < 3:
            print('invalid input')
            continue
        print(robot.set_position(*coords))


def position_playback():
    uart0 = UART(0, baudrate=BAUDRATE, tx=Pin(0), rx=Pin(1))


    directionpin = Pin(2, Pin.OUT)
    directionpin.value(0)

    robot = Robot(AX12(uart0, 0x1, directionpin),
                  AX12(uart0, 0x2, directionpin),
                  AX12(uart0, 0x3, directionpin),
                  AX12(uart0, 0x4, directionpin))

    command_button = Pin(3, Pin.IN, Pin.PULL_UP)
    prev_command = 1

    positions = []
    index = 0

    while 1:
        utime.sleep_ms(100)
        command = command_button.value()

        if not command:
            if prev_command:
                robot.disable_torque()
                positions = []

            value =  robot.get_joint_values()
            print('Recorded', value)
            positions.append(value)

        else:
            if not positions:
                continue

            if index >= len(positions):
                index = 0
                for joint in robot.joints:
                    joint.write_speed(100)
            else:
                for joint in robot.joints:
                    joint.write_speed(0)

            value = positions[index]
            print('Replaying', value)
            robot.set_joint_values(value)
            index += 1

        prev_command = command

if __name__ == '__main__':
    position_playback()
    # test_robot()
    # test_servo()

