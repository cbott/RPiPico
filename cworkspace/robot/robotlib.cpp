#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <math.h>
#include "AX12.h"
#include "robotlib.h"

// Nomenclature definition
// angle = radians, joint angle in robot coordinate frame
// coordinate = xyz (cartesian) coordinates
// value = raw servo 0-1023 position

Robot::Robot(AX12 * servo1, AX12 * servo2, AX12 * servo3, AX12 * servo4){
  joints[1] = servo1;
  joints[2] = servo2;
  joints[3] = servo3;
  joints[4] = servo4;

  for(int jointnum=0; jointnum<NUMJOINTS; ++jointnum){
    joints[jointnum]->write_cw_limit(JOINT_LIMITS[jointnum][0]);
    joints[jointnum]->write_ccw_limit(JOINT_LIMITS[jointnum][1]);
    joints[jointnum]->write_speed(100);
  }
}

// class Robot:
//     def __init__(self, servo1: AX12, servo2: AX12, servo3: AX12, servo4: AX12) -> None:
//         self.prev_angles = Pose(pi/2, 2, -pi/2, -pi/2)
//         self.set_joint_angles(self.prev_angles)

//     def set_joint_value(self, joint_num: int, position: int) -> None:
//         """ Set joint position directly as Servo value, after rounding to nearest int """
//         # TODO: Add bounds checking or read error flag after write
//         # maybe write_position needs to return a bool and we pass that along?
//         self.joints[joint_num].write_position(position)
//         # TODO: find a way to cleanly merge with next method, maybe passing None for a list value skips it?

//     def set_joint_values(self, values: list) -> None:
//         # TODO: set speeds here too? Probably not but then maybe a standalone speed setting method?
//         for i in range(len(self.joints)):
//             self.set_joint_value(i, values[i])

//     def set_joint_angles(self, angles: Pose, speed: int = 100) -> bool:
//         """ Set angles in radians """
//         # Determine motion speeds
//         # TODO: should we make a standard "angle list" type to pass around? Probably not but could still hold in a list
//         n = len(angles)  # TODO: should use class constant num joints
//         deltas = [abs(angles[i] - self.prev_angles[i]) for i in range(n)]
//         max_delta = max(deltas)
//         if max_delta > 0:
//             for joint_num in range(n):
//                 self.joints[joint_num].write_speed(int(speed * deltas[joint_num] / max_delta))
//         self.prev_angles = angles

//         # Convert angle to servo counts
//         # TODO: determine if 0.29 is accurate
//         COUNTS_PER_RADIAN = 197.571654  # 0.29 degrees per count
//         values = [
//             (angles.theta0 * COUNTS_PER_RADIAN) + 201,
//              822 - (angles.theta1 * COUNTS_PER_RADIAN),  # Or (angles.theta1 * COUNTS_PER_RADIAN) + 201 if we fix servo direction
//              512 - (angles.theta2 * COUNTS_PER_RADIAN),  # Or (angles.theta2 * COUNTS_PER_RADIAN) + 512
//              512 - (angles.theta3 * COUNTS_PER_RADIAN)  # Or (angles.theta3 * COUNTS_PER_RADIAN) + 512
//         ]
//         values = [int(round(p, 0)) for p in values]

//         for joint in range(self.NUM_JOINTS):
//             limits = self.JOINT_LIMITS[joint]
//             if not limits[0] <= values[joint] <= limits[1]:
//                 return False

//         self.set_joint_values(values)
//         return True

//     @classmethod
//     def inverse_kinematics(cls, x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False) -> Pose:
//         """ Determine necessary joint angles to reach a position """
//         # Solve for joint 0 angle
//         theta0 = atan2(y, x)
//         if invert_base:
//             # When we want to "reach over backwards" the base points the opposite direction of the desired point
//             theta0 += pi
//         # Constrain base to the valid servo range of -pi/3 to 8pi/6
//         # We may still end up in an invalid range, caller must ensure this is not the case
//         if theta0 < -pi/3:
//             theta0 += 2*pi
//         if theta0 > 8*pi/6:
//             theta0 -= 2*pi

//         # Translate end effector coordinates to joint 3 coordinates
//         d_z = cls.L3 * sin(end_angle)
//         d_r = cls.L3 * cos(end_angle)
//         x = x - d_r * cos(theta0)
//         y = y - d_r * sin(theta0)
//         z = z - d_z

//         # Solve for joint 2 angle
//         dist_squared = x**2 + y**2 + z**2
//         # solve for cos(theta2) using law of cosines
//         cos_theta2 = (dist_squared - cls.L1**2 - cls.L2**2) / (2 * cls.L1 * cls.L2)
//         if not -1 <= cos_theta2 <= 1:
//             raise ValueError("Target position {} not achievable".format((x, y, z)))
//         sin_theta2 = -(1 - cos_theta2**2)**0.5
//         if invert_elbow:
//             sin_theta2 *= -1
//         theta2 = atan2(sin_theta2, cos_theta2)

//         # Solve for joint 1 angle
//         r = (x**2 + y**2)**0.5
//         alpha = atan2(z, r)
//         beta = atan2(cls.L2 * sin(theta2), cls.L2 * cos(theta2) + cls.L1)
//         theta1 = alpha - beta

//         # Make corrections for flipped base
//         if invert_base:
//             # Base has been set to face away from the target point, reflect remaining joints about z-r plane
//             theta1 = pi - theta1
//             theta2 *= -1

//         # Solve for joint 3 angle
//         # TODO: make this play nice with inverted base
//         theta3 = end_angle - theta1 - theta2
//         # TODO: do this with mod math
//         if theta3 < -pi:
//             theta3 += 2*pi
//         if theta3 > pi:
//             theta3 -= 2*pi

//         return Pose(theta0, theta1, theta2, theta3)

//     @classmethod
//     def numerical_ik_solver(cls):
//         pass

//     def disable_torque(self) -> None:
//         for joint in self.joints:
//             joint.write_torque_enable(False)

//     def get_joint_values(self) -> list:
//         return [joint.read_position() for joint in self.joints]

//     def set_position(self, x: float, y: float, z:float, end_angle: float = 0, invert_base: bool = False, invert_elbow: bool = False, speed: int = 100) -> bool:
//         """ Move the end effector to the specified cartesian coordinates

//         end_angle: desired angle of the end effector relative to horizontal, in radians
//         invert_base: set to True to "reach over backward" to hit the specified location
//         invert_elbow: set to True to reach the specified locaton with inverted elbow geometry
//         speed: maximum joint speed, servo units

//         returns: True if position is set, False if position is unachievable
//         """
//         angles = self.inverse_kinematics(x, y, z, end_angle, invert_base, invert_elbow)
//         print('Setting joint angles of {}, {}, {}, {} degrees'.format(angles.theta0*180/pi, angles.theta1*180/pi, angles.theta2*180/pi, angles.theta3*180/pi))
//         return self.set_joint_angles(angles, speed)

