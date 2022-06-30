#ifndef ROBOTLIB_H
#define ROBOTLIB_H

// Doing this as a define just so it's explicit where "4" is being used for this purpose
#define NUMJOINTS 4

class Robot {
private:
  // TODO: is this nonsense or does array of pointers to objects make sense?
  AX12* joints[NUMJOINTS];
  // TODO: static?
  const int JOINT_LIMITS[NUMJOINTS][2] = {
        {0, 1023},
        {200, 830},
        {45, 1000},
        {45, 1000}
  };
  // Beam lengths [mm]
  static const int L1 = 120;
  static const int L2 = 115;
  static const int L3 = 58;

public:
  Robot(AX12 * servo1, AX12 * servo2, AX12 * servo3, AX12 * servo4);
};

#endif
