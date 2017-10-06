/// This file defines all the stuff related to the interface between
/// ros_controllers and the hardware interface

#include "lk_rover/lk_rover.h"

/// TwinJoints provides a way to convert one encoder/PWM pair into two of them,
/// more or less
/// Theoretically if everything worked perfectly, both of the encoders would
/// read the same value at the same position, and the motors would stay in 
/// perfect sync so you'd just feed the same PWM value into both of them
/// Of course, Santa isn't real so instead you do some PID stuff that'll cause
/// the joints to tend towards matching each other, and you also have to do 
/// some work to scale the values from ROS values to values usable by the Arduino

TwinJoints::TwinJoints(const ActuatorConfigs &ac):
  a(ac.left), b(ac.right), diffGain(ac.diffGain), length(ac.length), lastA(0.0), lastB(0.0) {}

/// This gets data from the encoders to ROS
/// ROS assumes all linear measurements are in meters (and all radial 
/// measurements are in radians), so there's some code to convert from the
/// encoder values to meters
double TwinJoints::getProcessedEncoder(double inA, double inB) {
  // store the values so that they can be used for computing the necessary
  // PWMs
  lastA = (inA - a.min)/(a.max - a.min);
  lastB = (inB - b.min)/(b.max - b.min);

  // ROS_INFO("a b %lf %lf", lastA, lastB);
  // use the average as the position of the virtual actuator
  // returns the virtual distance in meters
  return length*(lastA + lastB)/2;
}

/// This sends data from ROS to the encoders doing that thing I said they'd
/// do two paragraphs up
void TwinJoints::getProcessedPwms(double pwm, double &outA, double &outB) {
  // basically adjust the pwm output based on the discrepancy between the two
  // measured distances
  // ROS_INFO("pwm %lf", pwm);
  double diff = lastA - lastB;
  if (std::abs(diff) <= 0.001) diff = 0.0;
  outA = a.gain * (pwm - diffGain*diff);
  outB = b.gain * (pwm + diffGain*diff);
  // ROS_INFO("diff %lf %lf %lf %lf", diffGain, diff, outA, outB);
}

double dummyVal = 0.0;

/// This is the main interface between ROS and the hardware interface
/// It mostly just calls the hardware interface and computes derivatives
/// and integrals for the PID stuff
LKRover::LKRover(std::shared_ptr<LKHW> hw_, ActuatorConfigs& dump, ActuatorConfigs& ladder):
    /// Initialize everything
    virtualDump(dump),
    virtualLadder(ladder),
    wheelAccels(),
    wheelPoss(),
    wheelVels(),
    wheelPwms(),
    dumpPos(0.0),
    dumpVel(0.0),
    dumpAccel(0.0),
    dumpPwm(0.0),
    ladderPos(0.2),
    ladderVel(0.0),
    ladderAccel(0.0),
    ladderPwm(0.0),
    hw(hw_),
    lastTime(ros::Time::now()),
    spin(0.0),
    flap(0.0),
    killed(false) {
  
  /// This stuff's kind of important; ROS has a really convoluted system
  /// where you set up joint state handles and joint handles
  /// and it'll talk to those instead of you
  /// It's weird because ROS is weird
  for (int i = 0; i < kNumWheels; ++i) {
    /// But yeah, this sets up a number of different joint state handles
    /// that'll then be able to read from the wheelPoss, wheelVels, and wheelAccels
    /// array elements
    /// Each joint state is named from the kWheelNames array
    /// with names that have to match the names in the URDF model
    /// Because ROS is weird and uses the model for some calculations
    auto jsh = hardware_interface::JointStateHandle(
        kWheelNames[i], &wheelPoss[i], &wheelVels[i], &wheelAccels[i]);
    /// It also sets up a joint handle that's connected to the wheelPwms array elements
    /// And it'll write PWM values to that array
    auto jh = hardware_interface::JointHandle(jsh, &wheelPwms[i]);

    /// We register the joint state handle with the joint state interface
    /// and the joint handle to the velocity joint interface
    /// This'll matter near the end of this function
    /// where we register all the interfaces
    /// because ROS is weird
    jsi.registerHandle(jsh);
    vji.registerHandle(jh);
  }

  /// Do basically the same thing for a crap ton of other stuff
  auto dsh = hardware_interface::JointStateHandle(
      "dump", &dumpPos, &dumpVel, &dumpAccel);
  auto dh = hardware_interface::JointHandle(dsh, &dumpPwm);
  jsi.registerHandle(dsh);
  vji.registerHandle(dh);

  auto lsh = hardware_interface::JointStateHandle(
      "ladder", &ladderPos, &ladderVel, &ladderAccel);
  auto lh = hardware_interface::JointHandle(lsh, &ladderPwm);
  jsi.registerHandle(lsh);
  vji.registerHandle(lh);

  auto ssh = hardware_interface::JointStateHandle(
      "spin", &dummyVal, &dummyVal, &dummyVal);
  auto sjh = hardware_interface::JointHandle(ssh, &spin);
  eji.registerHandle(sjh);

  auto fsh = hardware_interface::JointStateHandle(
      "flap", &dummyVal, &dummyVal, &dummyVal);
  auto fjh = hardware_interface::JointHandle(fsh, &flap);
  pji.registerHandle(fjh);

  /// Then register all the interfaces so ROS can use them
  registerInterface(&vji);
  registerInterface(&jsi);
  registerInterface(&pji);
  registerInterface(&eji);

  // initialize all the PWM values
  double dumpA, dumpB, ladderA, ladderB;
  hw->getCount(wheelPoss, dumpA, dumpB, ladderA, ladderB);
  dumpPos = virtualDump.getProcessedEncoder(dumpA, dumpB);
  // ladderPos = virtualDump.getProcessedEncoder(ladderA, ladderB);
}

/// We don't need to do anything when this interface is shut down,
/// so the destructor, which gets called when this object is destroyed,
/// is empty
LKRover::~LKRover() {
}

/// Basically keep track of whether or not something told us to kill the motors
void LKRover::killMotors() {
  killed = true;
}

/// Or not to kill the motors
void LKRover::unkillMotors() {
  killed = false;
}

/// Then when whatever tells us to send out the PWM values, we either send
/// them out, or we send out zeroes if the motors should be killed
void LKRover::write() {
  double dumpA, dumpB, ladderA, ladderB;
  /// Convert the single dumpPwm value into two PWM values using one of the
  /// TwinJoint object things
  virtualDump.getProcessedPwms(dumpPwm, dumpA, dumpB);
  /// And then do the same for the other pair of motors
  virtualLadder.getProcessedPwms(ladderPwm, ladderA, ladderB);
  // ROS_INFO("pwms %lf %lf %lf %lf %lf %lf",  dumpA, dumpB, ladderA, ladderB, dumpPwm, ladderPwm);

  if (killed) {
    /// Clear all the PWM values if the motors should be stopped
    dumpA = 0.0;
    dumpB = 0.0;
    ladderA = 0.0;
    ladderB = 0.0;
    spin = 0.0;
    flap = 0.0;
    for (auto &pwm: wheelPwms) {
      pwm = 0.0;
    }
  }
  /// And then send the values
  hw->setPWMs(wheelPwms, dumpA, dumpB, ladderA, ladderB, spin, flap);
}

/// This reads data from the hardware interface
void LKRover::read() {
  std::array<double, kNumWheels> oldPos = wheelPoss;
  std::array<double, kNumWheels> oldVels = wheelVels;
  double oldDumpPos = dumpPos, oldLadderPos = ladderPos;
  double oldDumpVel = dumpVel, oldLadderVel = ladderVel;

  double dumpA, dumpB, ladderA, ladderB;

  hw->getCount(wheelPoss, dumpA, dumpB, ladderA, ladderB);
  dumpPos = virtualDump.getProcessedEncoder(dumpA, dumpB);
  ladderPos = virtualLadder.getProcessedEncoder(ladderA, ladderB);

  ros::Time curTime = ros::Time::now();
  /// This is a lambda expression
  /// It's basically a function that's also an object
  /// This approximates the derivative by dividing the change in value
  /// by the change in time
  auto d = [&](double newPos, double old) -> double {
    return (newPos - old)/(curTime - lastTime).toSec();
  };
  for (int i = 0; i < kNumWheels; i++) {
    wheelVels[i] = d(wheelPoss[i], oldPos[i]);
  }
  for (int i = 0; i < kNumWheels; i++) {
    wheelAccels[i] = d(wheelVels[i], oldVels[i]);
  }

  dumpVel = d(dumpPos, oldDumpPos);
  ladderVel = d(ladderPos, oldLadderPos);
  dumpAccel = d(dumpVel, oldDumpVel);
  ladderAccel = d(ladderVel, oldLadderVel);

  lastTime = curTime;
}
