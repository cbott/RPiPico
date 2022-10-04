#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <sstream>
#include <iostream>
#include <math.h>
#include "AX12.h"
#include "robotlib.h"

// Nomenclature definition
// angle = radians, joint angle in robot coordinate frame
// coordinate = xyz (cartesian) coordinates
// value = raw servo 0-1023 position

Robot::Robot(AX12 *servo1, AX12 *servo2, AX12 *servo3, AX12 *servo4)
    : prev_angles{M_PI_2, 2.0, -M_PI_2, -M_PI_2},
      joints{servo1, servo2, servo3, servo4}
{
  // Initialize servo speed and angle limits
  // Doing this in 3 sections because we seem to get comm errors when setting
  // two things in a row on the same servo
  for(int jointnum=0; jointnum<NUMJOINTS; ++jointnum){
    joints[jointnum]->write_cw_limit(JOINT_LIMITS[jointnum][0]);
  }
  sleep_ms(20);
  for(int jointnum=0; jointnum<NUMJOINTS; ++jointnum){
    joints[jointnum]->write_ccw_limit(JOINT_LIMITS[jointnum][1]);
  }
  sleep_ms(20);
  for(int jointnum=0; jointnum<NUMJOINTS; ++jointnum){
    joints[jointnum]->write_speed(100);
  }

  // Move servos to known starting position
  set_joint_angles(prev_angles);
}

/**
 * Set joint positions directly as Servo value
 *
 * @param values array of joint values to set
 */
void Robot::set_joint_values(uint16_t values[]){
  // TODO: Add bounds checking or read error flag after write
  // TODO: Should we be doing speed calculations here rather than when setting angles?
  for(int i=0; i < NUMJOINTS; ++i){
    std::cout << "Setting Joint " << i << " to value " << values[i] << std::endl;
    joints[i]->write_position(values[i]);
  }
}

/**
 * Set joint angles in radians
 *
 * @param angles array of joint angles to set (radians)
 * @param speed maximum speed for any joint, scale slower joints proportionally
 * @return whether specified angles can be set within physical joint limits
 */
bool Robot::set_joint_angles(float angles[], int speed){
  // TODO: should we make a standard "angle list" struct?
  // Determine motion speeds
  float deltas[4];
  float max_delta = 0;
  for(int i=0; i < NUMJOINTS; ++i){
    float d = abs(angles[i] - prev_angles[i]);
    deltas[i] = d;
    if(d > max_delta) max_delta = d;
  }
  if(max_delta > 0){
    for(int joint_num = 0; joint_num < NUMJOINTS; ++joint_num){
      // TODO: verify type casting here
      uint16_t joint_speed = (uint16_t) (speed * deltas[joint_num] / max_delta);
      std::cout << "Setting Joint " << joint_num << " to speed " << joint_speed << std::endl;
      joints[joint_num]->write_speed(joint_speed);
    }
  }
  for(int i=0; i < NUMJOINTS; ++i){
    prev_angles[i] = angles[i];
  }

  // Convert angle to servo counts
  // TODO: determine if 0.29 is accurate
  float COUNTS_PER_RADIAN = 197.571654; // 0.29 degrees per count
  uint16_t values[NUMJOINTS] = {
    (uint16_t) lround((angles[0] * COUNTS_PER_RADIAN) + 201),
    (uint16_t) lround(822 - (angles[1] * COUNTS_PER_RADIAN)),  // Or (angles[1] * COUNTS_PER_RADIAN) + 201 if we fix servo direction
    (uint16_t) lround(512 - (angles[2] * COUNTS_PER_RADIAN)),  // Or (angles[2] * COUNTS_PER_RADIAN) + 512
    (uint16_t) lround(512 - (angles[3] * COUNTS_PER_RADIAN))   // Or (angles[3] * COUNTS_PER_RADIAN) + 512
  };

  // Check calculated values against absolute physical limits
  for(int i=0; i < NUMJOINTS; ++i){
    if(values[i] < JOINT_LIMITS[i][0] || values[i] > JOINT_LIMITS[i][1]){
      std::cout << "Setting Joint " << i << " to " << values[i] << " violates bounds " << JOINT_LIMITS[i][0] << ", " << JOINT_LIMITS[i][1] << std::endl;
      return false;
    }
  }

  set_joint_values(values);
  return true;
}

/**
 * Move the end effector to the specified cartesian coordinates
 *
 * @param x X coordinate [mm]
 * @param y Y coordinate [mm]
 * @param Z Z coordinate [mm]
 * @param end_angle desired angle of the end effector relative to horizontal, in radians
 * @param invert_base set to True to "reach over backward" to hit the specified location
 * @param invert_elbow set to True to reach the specified locaton with inverted elbow geometry
 * @param speed maximum joint speed, servo units
 * @return True if position is set, False if position is unachievable
 */
bool Robot::set_position(float x, float y, float z, float end_angle, bool invert_base, bool invert_elbow, int speed){
  float angles[4];
  bool valid = inverse_kinematics(angles, x, y, z, end_angle, invert_base, invert_elbow);
  if(!valid){
    std::cout << "Unable to calculate valid solution for requested position " << x << " " << y << " " << z << std::endl;
    return false;
  }

  std::cout << "Setting joint angles of ";
  std::cout << angles[0] * 180.0 / M_PI << ", ";
  std::cout << angles[1] * 180.0 / M_PI << ", ";
  std::cout << angles[2] * 180.0 / M_PI << ", ";
  std::cout << angles[3] * 180.0 / M_PI << std::endl;

  return set_joint_angles(angles, speed);
}

/**
 * Determine necessary joint angles to reach a position
 *
 * @param result Result array of 4 angles (radians) to be filled in with calculated pose
 * @param x Target x coordinate (mm)
 * @param y Target y coordinate (mm)
 * @param z Target z coordinate (mm)
 * @param end_angle Target angle of end effector relative to horizontal (degrees)
 * @param invert_base Reach over backward
 * @param invert_elbow Position elbow joint with opposite concavity
 * @return true if requested angle is achievable, false if it is not
 */
bool Robot::inverse_kinematics(float result[], float x, float y, float z,
                               float end_angle, bool invert_base, bool invert_elbow){
  // Solve for joint 0 angle
  float theta0 = atan2(y, x);
  if(invert_base){
    // When we want to "reach over backwards" the base points the opposite direction of the desired point
    theta0 += M_PI;
  }
  // Constrain base to the valid servo range of -pi/3 to 8pi/6
  // We may still end up in an invalid range, caller must ensure this is not the case
  if(theta0 < -M_PI / 3.0){
    theta0 += 2.0 * M_PI;
  }
  if(theta0 > 8.0 * M_PI / 6.0){
    theta0 -= 2.0 * M_PI;
  }

  // Translate end effector coordinates to joint 3 coordinates
  float d_z = L3 * sin(end_angle);
  float d_r = L3 * cos(end_angle);
  x = x - d_r * cos(theta0);
  y = y - d_r * sin(theta0);
  z = z - d_z;

  // Solve for joint 2 angle
  float dist_squared = pow(x, 2) + pow(y, 2) + pow(z, 2);
  // Solve for cos(theta2) using law of cosines
  float cos_theta2 = (dist_squared - pow(L1, 2) - pow(L2, 2)) / (2.0 * L1 * L2);
  if(cos_theta2 < -1 || cos_theta2 > 1){
    // Position not achievable
    return false;
  }
  float sin_theta2 = -pow((1.0 - pow(cos_theta2, 2)), 0.5);
  if(invert_elbow){
    sin_theta2 *= -1;
  }
  float theta2 = atan2(sin_theta2, cos_theta2);

  // Solve for joint 1 angle
  float r = pow((pow(x, 2) + pow(y, 2)), 0.5);
  float alpha = atan2(z, r);
  float beta = atan2(L2 * sin(theta2), L2 * cos(theta2) + L1);
  float theta1 = alpha - beta;

  // Make corrections for flipped base
  if(invert_base){
    // Base has been set to face away from the target point, reflect remaining joints about z-r plane
    theta1 = M_PI - theta1;
    theta2 *= -1;
  }
  // Solve for joint 3 angle
  // TODO: make this play nice with inverted base
  float theta3 = end_angle - theta1 - theta2;
  // TODO: do this with mod math
  if(theta3 < -M_PI){
    theta3 += 2.0 * M_PI;
  }
  if(theta3 > M_PI){
    theta3 -= 2.0 * M_PI;
  }

  result[0] = theta0;
  result[1] = theta1;
  result[2] = theta2;
  result[3] = theta3;
  return true;
}

void Robot::run_interactive(uart_inst_t *uart){
  // Interface with
  // picocom /dev/ttyACM0 -b 115200 --echo --omap crcrlf
  std::cout << "Enter Command" << std::endl;

  // Option 1: the easy way
  char cmd;
  std::string line;

  while(1){
    std::getline(std::cin, line);
    std::stringstream linestream(line);
    if(!(linestream >> cmd)){
      std::cout << "Invalid Command " << linestream.str() << std::endl;
      continue;
    }

    if(cmd == 'H'){
      std::cout << "Home" << std::endl;
    }
    else if(cmd == 'J'){
      // TODO: Allow setting single joints
      // This could be implemented like g-code, but that's a bit clunky
      // J A<#> B<#> C<#> D<#>
      // alternatively
      // J - - # -
      uint16_t j1, j2, j3, j4;
      if(!(linestream >> j1 >> j2 >> j3 >> j4)){
        std::cout << "Invalid Parameters" << std::endl;
        continue;
      }
      std::cout << "Setting " << j1 << " " << j2 << " " << j3 << " " << j4 << std::endl;
      uint16_t vals[] = {j1, j2, j3, j4};
      set_joint_values(vals);
    }
    else if(cmd == 'A'){
      float a1, a2, a3, a4;
      if(!(linestream >> a1 >> a2 >> a3 >> a4)){
        std::cout << "Invalid Parameters" << std::endl;
        continue;
      }
      uint16_t speed;
      if(!(linestream >> speed)){
        speed = DEFAULT_SPEED;
      }
      std::cout << "Setting " << a1 << " " << a2 << " " << a3 << " " << a4 << " " << speed << std::endl;
      float vals[] = {a1, a2, a3, a4};
      set_joint_angles(vals, speed);
    }
    else if(cmd == 'G'){
      int x, y, z;
      if(!(linestream >> x >> y >> z)){
        std::cout << "Invalid Parameters" << std::endl;
        continue;
      }
      std::cout << "Setting " << x << " " << y << " " << z << std::endl;
    }
    else if(cmd == 'E'){
      // TODO: we could take an optional param to specify which
      // joint we are interested in?
      std::cout << "Errors" << std::endl;
      for(int i=0; i<NUMJOINTS; ++i){
        std::cout << unsigned(joints[i]->device_status_error.error_byte) << std::endl;
      }
    }
    else if(cmd == 'R'){
      std::cout << "Read Position" << std::endl;
      for(int i=0; i<NUMJOINTS; ++i){
        std::cout << joints[i]->read_position() << " ";
      }
      std::cout << std::endl;
    }
    else {
      std::cout << "\nUnrecognized command " << +cmd << std::endl;
    }

  }
}
