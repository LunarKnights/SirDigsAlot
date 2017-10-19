#include <memory>

#include "lk_rover/common.h"

#include "lk_rover/lk_rover_hw.h"

/// This is the hardware interface for the actual robot (in contrast to the
/// gazebo interface)
/// This just forwards commands to the Arduino ROS node thing,
/// which then sends them across USB to the Arduino, which then does
/// the actual work
LKRoverHW::LKRoverHW(ros::NodeHandle nh) : nh(nh)
{
  /// Set up the stuff for the two topics we care about
  pubPwm = nh.advertise<lk_rover::AllPWMs>("pwms", 1);
  subEncoders = nh.subscribe("encoders", 1, &LKRoverHW::encoderCb, this);
}

/// This combines all the PWM stuff into a single message that gets sent to
/// the ROS Arduino node
void LKRoverHW::setPWMs(const std::array<double, kNumWheels>& pwms, double dumpA, double dumpB, double ladderA,
                        double ladderB, double spin, double flap)
{
  lk_rover::AllPWMs allPwms;
  allPwms.bucket_left = -dumpA;
  allPwms.bucket_right = -dumpB;

  allPwms.bucket_spin = spin;
  allPwms.bucket_flap = flap;

  allPwms.ladder_left = -ladderA;
  allPwms.ladder_right = -ladderB;

  allPwms.front_left = pwms[0];
  allPwms.back_left = pwms[1];
  allPwms.front_right = -pwms[2];
  allPwms.back_right = -pwms[3];

  pubPwm.publish(allPwms);
}

/// This gets the last sets of values saved from the data this node's received
/// from the ROS Arduino node
void LKRoverHW::getCount(std::array<double, kNumWheels>& encoderValsInRadians, double& dumpA, double& dumpB,
                         double& ladderA, double& ladderB)
{
  /// First it locks the mutex
  /// This is necessary because we don't know when the Arduino node will
  /// send data and invoke the callback function that sets the encoder values
  /// By locking the mutex both here and in the callback function, we make
  /// sure that only one of these functions accesses the encoder values at a
  /// time, avoiding a race condition and all kinds of potential weird behavior
  encoderLock.lock();
  /// Copy all the values into the output
  encoderValsInRadians[0] = kEncoderToRadians * lastEncoderData.front_left_enc;
  encoderValsInRadians[1] = kEncoderToRadians * lastEncoderData.back_left_enc;
  encoderValsInRadians[2] = kEncoderToRadians * lastEncoderData.front_right_enc;
  encoderValsInRadians[3] = kEncoderToRadians * lastEncoderData.back_right_enc;

  dumpA = lastEncoderData.bucket_left_enc;
  dumpB = lastEncoderData.bucket_right_enc;

  ladderA = lastEncoderData.ladder_left_enc;
  ladderB = lastEncoderData.ladder_right_enc;

  /// Unlock the mutex so something else can access the data
  encoderLock.unlock();
}

/// This is the callback called by ROS when it receives a message from the
/// Arduino node
/// It just copies the message into lastEncoderData
void LKRoverHW::encoderCb(const lk_rover::AllEncoders& encoders)
{
  /// It tries to lock the mutex, and stores the data into lastEncoderData
  /// if the mutex lock is successful
  /// Otherwise, it doesn't do anything
  /// It could try to wait until the mutex is available again and then store the
  /// data, but it seems like losing one frame of data every once in a while
  /// is better than storing possibly stale data
  if (encoderLock.try_lock())
  {
    lastEncoderData = encoders;
    encoderLock.unlock();
  }
}

/// This is a helper function used in initialization to wait until the Arduino
/// ROS topics are available
/// This takes about 13 seconds for unknown reasons
void LKRoverHW::waitForSerial()
{
  ROS_INFO("waiting for serial...");
  auto msg = ros::topic::waitForMessage<lk_rover::AllEncoders>("encoders", ros::Duration(30));
  while (!msg)
  {
    ROS_WARN("attempt to get encoder message failed");
    msg = ros::topic::waitForMessage<lk_rover::AllEncoders>("encoders", ros::Duration(30));
  }
  ROS_INFO("serial start detected");
}
