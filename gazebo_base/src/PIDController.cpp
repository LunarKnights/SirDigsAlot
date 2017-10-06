#include "ros/ros.h"
#include "ros/console.h"

#include "tf2/utils.h"

#include "PIDController.h"

#include "BaseController.h"

void PIDController::pidControl() {
  // desiredForce = idx < 2 ? -5.0f : 5.0f;
  // desiredForce = 5.0f;

  // ROS_INFO("%d: %f", idx, parent->vels[idx]);

  // TODO: PID code here
  // currently doing bang-bang control with hysteresis
  float delta = curVel - desiredVel;
  if (std::abs(delta) < 0.2f) {
  } else if (delta > 0.0f) {
    desiredForce = 7.0f;
  } else {
    desiredForce = -7.0f;
  }

  setForce();
}

void PIDController::setForce() {
  if (std::abs(desiredForce - curForce) > 0.001f) {
    if (nForceApplications > kMaxForceNum) {
      // clear forces
      clearForce();
      curForce = 0.0f;
    }
    auto deltaForce = desiredForce - curForce;
    if (addForce(deltaForce)) {
      ++nForceApplications;
      curForce += deltaForce;
    } else {
      ROS_WARN("PIDController::addForce(%d, %f) failed", idx, deltaForce);
    }
  }
}

void PIDController::clearForce() {
  gazebo_msgs::JointRequest cjf;
  cjf.request.joint_name = kJointNames[idx];
  if (!parent -> clearJoints.call(cjf)) {
    ROS_WARN("PIDController::clearForce(%d) failed", idx);
  }
}

bool PIDController::addForce(float f) {
  gazebo_msgs::ApplyJointEffort aje;
  aje.request.joint_name = kJointNames[idx];
  aje.request.effort = f;
  aje.request.duration = ros::Duration(-0.1f);
  if (!parent -> moveJoints.call(aje)) {
    return false;
  } else if (!aje.response.success) {
    ROS_WARN("PIDController::addForce(%d, %f): aje request returned with %s",
        idx, f, aje.response.status_message.c_str());
  }
  return aje.response.success;
}
