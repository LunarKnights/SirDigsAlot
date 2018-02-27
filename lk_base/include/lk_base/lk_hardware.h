#ifndef LK_BASE_LK_HARDWARE_H
#define LK_BASE_LK_HARDWARE_H

#include "ros/ros.h"

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket.h"
#include "wpilib/CanTalonSRX.h"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

const int kNumWheels = 4;

namespace lk_base
{

class LKHardware : public hardware_interface::RobotHW
{
  struct Joint
  {
    double pos, accel, vel;
    double cmd;

    void updatePos(double newPos, double dt);
    void updateVel(double newVel, double dt);
  };
public:
  LKHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void registerInterfaces();
  void updateJointsFromHardware(double elapsed);
  void writeCommandsToHardware();

protected:
  Joint wheels[kNumWheels];
  Joint ladder, deposition;

  static bool canInterfaceInited;
  std::vector<std::shared_ptr<CanTalonSRX> > talons;

  // NOTE: husky_base seems to make do with only VJI and JSI
  // but that's probably because it doesn't have a deposition system like we do
  hardware_interface::VelocityJointInterface vji;
  hardware_interface::PositionJointInterface pji;
  hardware_interface::JointStateInterface jsi;
  hardware_interface::EffortJointInterface eji;
};

}

#endif
