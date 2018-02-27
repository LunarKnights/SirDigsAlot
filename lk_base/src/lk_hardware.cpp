#include "lk_base/lk_hardware.h"


// TODO: make sure this matches the names in lk_description
constexpr const char* kWheelNames[kNumWheels] = {
  "lf_wheel_joint", "lb_wheel_joint", "rf_wheel_joint", "rb_wheel_joint"
};

namespace lk_base
{

bool LKHardware::canInterfaceInited = false;

LKHardware::LKHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // initialize CAN interface if one's not already set up
  if (!canInterfaceInited)
  {
    ROS_INFO("initializing can socket interface...");
    can_talon_srx::CanSocketInterface::Init("can0");
    ROS_INFO("can socket interface initialized!");
  }
  // TODO: initialize hardware, read parameters, whatever
  registerInterfaces();
}

// Registers joint interfaces into the controller manager, so that lk_control
// can use them in its controllers
void LKHardware::registerInterfaces()
{
  for (int i = 0; i < kNumWheels; ++i)
  {
    hardware_interface::JointStateHandle jsh(kWheelNames[i],
        &wheels[i].pos, &wheels[i].vel, &wheels[i].accel);
    jsi.registerHandle(jsh);

    hardware_interface::JointHandle jh(jsh, &wheels[i].cmd);
    vji.registerHandle(jh);
  }
  // TODO: add joints for bucket and deposition system and whatever
  registerInterface(&vji);
  registerInterface(&pji);
  registerInterface(&jsi);
  registerInterface(&eji);
}

// Gets values from hardware and stores them in the Joint structs, so that
// the joint handles can access them
void LKHardware::updateJointsFromHardware(double elapsed)
{
  // TODO: update the joints as needed
  // NOTE: Talon appears to return position and velocity, so all that's missing
  // is the acceleration
}

// Send commands to hardware
void LKHardware::writeCommandsToHardware()
{
  // TODO
}

LKHardware::Joint::Joint():
  pos(0.0),
  vel(0.0),
  accel(0.0),
  cmd(0.0)
{
  // do nothing
}

}
