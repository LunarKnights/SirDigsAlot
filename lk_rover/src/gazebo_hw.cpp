#include <algorithm>
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/JointRequest.h"

#include "lk_rover/common.h"
#include "lk_rover/lk_rover.h"
#include "lk_rover/gazebo_hw.h"

constexpr double kForceTol = 0.001;
constexpr int kMaxForceNum = 8;

/// This is the hardware interface for controlling the gazebo model of the
/// robot
/// It uses different ROS services gazebo exposes to control the joints in
/// the simulation, as well as to generate data for the encoders by
/// getting the current orientation of the wheels
GazeboHW::GazeboHW() : curEfforts{}, numEfforts{}
{
}

bool GazeboHW::init(ros::NodeHandle& nh)
{
  /// Set up the different service objects, and fail if any of them aren't
  /// set up successfully
  mj = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort", true);
  cj = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces", true);
  gj = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties", true);

  if (!mj)
  {
    ROS_ERROR("unable to connect to /gazebo/apply_joint_effort");
    return false;
  }
  if (!cj)
  {
    ROS_ERROR("unable to connect to /gazebo/clear_joint_forces");
    return false;
  }
  if (!gj)
  {
    ROS_ERROR("unable to connect to /gazebo/get_joint_properties");
    return false;
  }
  return true;
}

/// This sends PWM commands to the virtal joints
void GazeboHW::setPWMs(const std::array<double, kNumWheels>& newEfforts, double dumpA, double dumpB, double ladderA,
                       double ladderB, double spin, double flap)
{
  // TODO: set up the nonwheel actuators
  ROS_INFO("GazeboHW::setPWMs %f %f %f %f", newEfforts[0], newEfforts[1], newEfforts[2], newEfforts[3]);
  for (int i = 0; i < kNumWheels; ++i)
  {
    /// First it clips the input wheel spin so that there's an upper limit on
    /// how fast the wheels can go, like it is in real life
    double clippedEfforts = newEfforts[i];
    if (newEfforts[i] > kMaxTorque)
    {
      clippedEfforts = kMaxTorque;
    }
    else if (newEfforts[i] < -kMaxTorque)
    {
      clippedEfforts = -kMaxTorque;
    }
    /// gazebo doesn't allow you to directly set the torque
    /// on any of the joints
    /// Instead you can tell it "torque this joint at 10 Newton-meters forever"
    /// or "remove every joint torque on this particular joint"
    /// and torque on that joint will be equal to the sum of all the joint
    /// torque commands you've sent it since you've last cleared it
    /// So if the wheel's already going at 9 Newton-meters, and you want it to
    /// be 10, you send a command for torquing it an addition 1 Newton-meter
    /// Over time there'll probably be some build up of numerical errors,
    /// along with some performance issues, so periodically we reset
    /// the forces by using the "clear joint forces" command
    /// and then applying the current force on it to make sure the joint's moving
    /// at the right torque
    /// We also avoid sending a command if the torque is below a certain tolerance,
    /// which reduces the amount of communication we need to do with gazebo

    /// We keep track of what the current torque in gazebo is using curEfforts
    /// and the number of torque orders gazebo has received with numEfforts
    if (std::abs(clippedEfforts - curEfforts[i]) > kForceTol)
    {
      /// This code does the clear force job call if we reach the threshold
      /// for a given joint
      if (numEfforts[i] > kMaxForceNum)
      {
        gazebo_msgs::JointRequest cjf;
        cjf.request.joint_name = kWheelNames[i];
        if (!cj.call(cjf))
        {
          ROS_WARN("GazeboHW::setPWMs: clear force %d failed", i);
          continue;
        }
        curEfforts[i] = 0.0;
      }
      /// This sends the new torque job to gazebo
      const auto deltaForce = clippedEfforts - curEfforts[i];
      gazebo_msgs::ApplyJointEffort aje;
      aje.request.joint_name = kWheelNames[i];
      aje.request.effort = deltaForce;
      /// A duration less than zero means indefinitely
      aje.request.duration = ros::Duration(-0.1f);
      if (!mj.call(aje))
      {
        continue;
      }
      else if (!aje.response.success)
      {
        ROS_WARN("GazeboHW::setPWMs: %d aje request failed with %s", i, aje.response.status_message.c_str());
        continue;
      }
      curEfforts[i] += deltaForce;
      ++numEfforts[i];
    }
  }
}

/// This is how we get the current encoder count from gazebo
/// We just copy the value from the joint properties
void GazeboHW::getCount(std::array<double, kNumWheels>& count, double& dumpA, double& dumpB, double& ladderA,
                        double& ladderB)
{
  // TODO: set up the nonwheel actuators
  for (auto i = 0; i < kNumWheels; ++i)
  {
    gazebo_msgs::GetJointProperties gjp;

    gjp.request.joint_name = kWheelNames[i];
    if (!gj.call(gjp))
    {
      ROS_WARN("GazeboHW::getCount get joints %d call failed", i);
      continue;
    }
    if (!gjp.response.success)
    {
      ROS_WARN("GazeboHW::getCount get joints %d call failed: %s", i, gjp.response.status_message.c_str());
      continue;
    }
    // ROS_INFO("position %d, %f", i, gjp.response.position[0]);
    count[i] = gjp.response.position[0];
  }
}
