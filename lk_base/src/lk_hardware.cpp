#include <memory>

#include "lk_base/lk_hardware.h"

// TODO: make sure this matches the names in lk_description
constexpr const char* kWheelNames[kNumWheels] = {
  "lf_wheel_joint", "lb_wheel_joint", "rf_wheel_joint", "rb_wheel_joint"
};

namespace lk_base
{

LKHardware::LKHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // TODO: initialize hardware, read parameters, whatever
  registerInterfaces();
}

// Registers joint interfaces into the controller manager, so that lk_control
// can use them in its controllers
void LKHardware::registerInterfaces()
{
  for (int i = 0; i < kNumWheels; ++i)
  {
    // TODO
  }
  // TODO
}

// Gets values from hardware and stores them in the Joint structs, so that
// the joint handles can access them
void LKHardware::updateJointsFromHardware(double elapsed)
{
  // TODO
}

// Send commands to hardware
void LKHardware::writeCommandsToHardware()
{
  // TODO
}

// Do calculations to calculate acceleration and velocity and whatever
void LKHardware::Joint::updatePos(double newPos, double dt)
{
  // TODO
}

// Do calculations to calculate acceleration and velocity and whatever
void LKHardware::Joint::updateVel(double newPos, double dt)
{
  // TODO
}

}
