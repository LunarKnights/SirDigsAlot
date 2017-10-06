#ifndef LK_HW_H
#define LK_HW_H

#include <array>

#include "lk_rover/common.h"

// there will be two classes that inherit from this class, one to talk to the
// Arduino, and one to talk to gazebo
// a parameter will be used to select between them when lk_rover_node is launched
class LKHW {
public:
  virtual void setPWMs(const std::array<double, kNumWheels>&,
      double dumpA, double dumpB, double ladderA, double ladderB, double spin, double flap) = 0;
  virtual void getCount(std::array<double, kNumWheels>&,
      double &dumpA, double &dumpB, double &ladderA, double &ladderB) = 0;
};

#endif
