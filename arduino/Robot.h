#pragma once

#include "Joint.h"
#include "MPU9250/src/MPU9250.h"

// All of the hardware on the robot
struct Robot {
  static const int N = 3;

  Joint joints[N];
  MPU9250 imu;

  void update(uint32_t ms) {
    for(int i = 0; i < N; i++) {
      joints[i].update(ms);
    }
  }
};
