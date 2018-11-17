#include "lk_base/lk_hardware.h"


// TODO: make sure this matches the names in lk_description
constexpr const char* kWheelNames[kNumWheels] = {
  "frontLeftWheelRotate", "backLeftWheelRotate", "frontRightWheelRotate", "backRightWheelRotate"
};

namespace lk_base
{

bool LKHardware::canInterfaceInited = false;

LKHardware::LKHardware(ros::NodeHandle nh, ros::NodeHandle private_nh): killed(false)
{
  // TODO: get topic name from param
  killMotors = nh.subscribe("kill_motors", 1, &LKHardware::killMotorsCb, this);

  // initialize CAN interface if one's not already set up
  if (!canInterfaceInited)
  {
    ROS_INFO("initializing can socket interface...");
    ctre::phoenix::platform::can::SetCANInterface("can0");
    ROS_INFO("can socket interface initialized!");
  }
  // TODO: initialize hardware, read parameters, whatever

    // TalonSRX * talon = new TalonSRX(1);
    // talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,kTimeoutMs);
    // talon->SetSensorPhase(true);
    // talon->ConfigNominalOutputForward(0,kTimeoutMs);
    // talon->ConfigNominalOutputReverse(0,kTimeoutMs);
    // talon->ConfigPeakOutputForward(1,kTimeoutMs);
    // talon->ConfigPeakOutputReverse(-1,kTimeoutMs);
    // talon->Config_kF(0,0.1097,kTimeoutMs);
    // talon->Config_kP(0,.22,kTimeoutMs);
    // talon->Config_kI(0,0,kTimeoutMs);
    // talon->Config_kD(0,0,kTimeoutMs);
    // talon->SetStatusFramePeriod(StatusFrame::Status_1_General_,5,kTimeoutMs);


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
//  registerInterface(&pji);
  registerInterface(&jsi);
//  registerInterface(&eji);
}

// Gets values from hardware and stores them in the Joint structs, so that
// the joint handles can access them
void LKHardware::updateJointsFromHardware(double elapsed)
{
  // TODO: update the joints as needed
  // NOTE: Talon appears to return position and velocity, so all that's missing
  // is the acceleration, which should probably be a little time-averaged
  // so that bad data doesn't mess it up too badly
  //talon->GetClosedLoopTarget()
//talon->GetSelectedSensorVelocity()
//talon->GetClosedLoopError()

}

// Send commands to hardware
void LKHardware::writeCommandsToHardware()
{
  if (killed)
  {
    // TODO: stop all motors
  }
  // TODO: write command to motors
  //    talon->Set(ControlMode::Velocity,targetVelocity_UnitsPer100ms);
  //    ctre::phoenix::unmanaged::FeedEnable(100);


}

void LKHardware::killMotorsCb(const std_msgs::Bool &msg)
{
  killed = msg.data;
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
