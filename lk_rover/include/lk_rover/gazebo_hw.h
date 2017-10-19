#ifndef LK_GAZEBO_HW_H
#define LK_GAZEBO_HW_H

#include "ros/ros.h"
#include "ros/console.h"

#include <array>

#include "lk_rover/lk_hw.h"

class GazeboHW : public LKHW
{
public:
  GazeboHW();
  bool init(ros::NodeHandle& nh);
  // TODO: wrap PWMs/encoder values in a struct
  void setPWMs(const std::array<double, kNumWheels>&, double dumpA, double dumpB, double ladderA, double ladderB,
               double spin, double flap);
  void getCount(std::array<double, kNumWheels>&, double& dumpA, double& dumpB, double& ladderA, double& ladderB);

private:
  ros::ServiceClient mj, cj, gj;

  std::array<double, kNumWheels> curEfforts;
  std::array<int, kNumWheels> numEfforts;
};

#endif
