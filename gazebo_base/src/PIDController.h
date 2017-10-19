#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "ros/ros.h"

#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/JointRequest.h"

#include "common.h"

constexpr int kMaxForceNum = 5;

class PIDController
{
public:
  float desiredVel, curVel;
  float desiredForce;

  int idx;

protected:
  float curForce;
  int nForceApplications;

  BaseController* parent;

public:
  PIDController(BaseController* p, int idx)
    : parent(p), idx(idx), nForceApplications(0), desiredVel(0.0f), curVel(0.0f), desiredForce(0.0f), curForce(0.0f)
  {
  }

  PIDController() : PIDController(nullptr, 0)
  {
  }
  PIDController(const PIDController& other) : PIDController(other.parent, other.idx)
  {
    desiredVel = other.desiredVel;
    curVel = other.curVel;
    desiredForce = other.desiredForce;
    curForce = other.curForce;
    nForceApplications = other.nForceApplications;
  }

  PIDController& operator=(const PIDController& other)
  {
    parent = other.parent;
    idx = other.idx;

    desiredVel = other.desiredVel;
    curVel = other.curVel;
    desiredForce = other.desiredForce;
    curForce = other.curForce;
    nForceApplications = other.nForceApplications;
  }

public:
  void pidControl();
  void setForce();
  void clearForce();
  bool addForce(float f);
};

#endif
