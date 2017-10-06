#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"

#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include <array>

#include "common.h"
#include "PIDController.h"

class BaseController {
  friend class PIDController;

protected:
  std::array<PIDController, kNumJoints> pidControllers;
  std::array<double, kNumJoints> vels;

  ros::ServiceClient &moveJoints, &clearJoints, &getJoints;

public:
  BaseController(ros::ServiceClient &mj, ros::ServiceClient &cj, ros::ServiceClient &gj);

  void ControllerCallback(const geometry_msgs::Twist& t);

  void PollJoints();

protected:
  void pidControl();
};

#endif
