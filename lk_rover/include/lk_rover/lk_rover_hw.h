#ifndef LK_ROVER_HW_H
#define LK_ROVER_HW_H

#include <mutex>

#include "ros/ros.h"

#include "lk_rover/AllPWMs.h"
#include "lk_rover/AllEncoders.h"

#include "lk_rover/lk_hw.h"

class LKRoverHW : public LKHW
{
public:
  LKRoverHW(ros::NodeHandle nh);

  void setPWMs(const std::array<double, kNumWheels>&, double dumpA, double dumpB, double ladderA, double ladderB,
               double spin, double flap);
  void getCount(std::array<double, kNumWheels>&, double& dumpA, double& dumpB, double& ladderA, double& ladderB);
  void waitForSerial();

private:
  void encoderCb(const lk_rover::AllEncoders&);
  ros::NodeHandle nh;
  ros::Publisher pubPwm;
  ros::Subscriber subEncoders;

  std::mutex encoderLock;
  lk_rover::AllEncoders lastEncoderData;
};

#endif
