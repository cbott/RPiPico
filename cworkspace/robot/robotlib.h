#ifndef ROBOTLIB_H
#define ROBOTLIB_H

// Doing this as a define just so it's explicit where "4" is being used for this purpose
#define NUMJOINTS 4
#define DEFAULT_SPEED 100

class Robot {
private:
  static constexpr int JOINT_LIMITS[NUMJOINTS][2] = {
        {0, 1023},
        {200, 830},
        {45, 1000},
        {45, 1000}
  };

  // (1023 counts / 300 degrees) * (180 degrees / PI radians)
  static constexpr float COUNTS_PER_RADIAN = 195.3786;

  // Beam lengths [mm]
  static const int L1 = 120;
  static const int L2 = 115;
  static const int L3 = 58;

  // Instance variables
  AX12 *joints[NUMJOINTS];
  float prev_angles[NUMJOINTS];

public:
  Robot(AX12 *servo1, AX12 *servo2, AX12 *servo3, AX12 *servo4);

  void set_joint_values(uint16_t values[]);
  bool set_joint_angles(float angles[], int speed = DEFAULT_SPEED);
  bool set_position(float x, float y, float z, float end_angle = 0, bool invert_base = false, bool invert_elbow = false, int speed = DEFAULT_SPEED);
  bool inverse_kinematics(float result[], float x, float y, float z, float end_angle = 0, bool invert_base = false, bool invert_elbow = false);
  void run_interactive(uart_inst_t *uart);
};

#endif
