#pragma once

#include "Joint.h"

// A Robot<N> is just a set of N joints
template<int N>
struct Robot {
  Joint joints[N];
};
