#pragma once

#include "Joint.h"

// All of the hardware on the robot
struct Robot {
  static const int N = 3;

  Joint joints[N];
  // TODO: MPU6050?
};
