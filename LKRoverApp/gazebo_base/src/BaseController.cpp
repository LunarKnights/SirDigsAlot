#include "ros/ros.h"
#include "ros/console.h"

#include "gazebo_msgs/GetJointProperties.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/Pose.h"

#include "BaseController.h"
#include "PIDController.h"

BaseController::BaseController(
    ros::ServiceClient &mj, ros::ServiceClient &cj, ros::ServiceClient &gj): 
      moveJoints(mj), clearJoints(cj), getJoints(gj) {
  for (int i = 0; i < kNumJoints; i++) {
    pidControllers[i] = PIDController(this, i);
  }
}

void BaseController::ControllerCallback(const geometry_msgs::Twist& t) {
  float left = (t.linear.x - t.angular.z*kWheelSeparation) / kWheelRadius;
  float right = (t.linear.x + t.angular.z*kWheelSeparation) / kWheelRadius;

  pidControllers[0].desiredVel = left;
  pidControllers[1].desiredVel = left;
  pidControllers[2].desiredVel = right;
  pidControllers[3].desiredVel = right;
}

void BaseController::PollJoints() {
  for (auto i = 0; i < kNumJoints; ++i) {
    gazebo_msgs::GetJointProperties gjp;

    gjp.request.joint_name = kJointNames[i];
    if (!getJoints.call(gjp)) {
      ROS_WARN("BaseController::PollJoints loop %d call failed", i);
      continue;
    }
    if (!gjp.response.success) {
      ROS_WARN("BaseController::PollJoints loop %d failed: %s", i, gjp.response.status_message.c_str());
      continue;
    }

    if (gjp.response.rate.size() >= 1) {
      vels[i] = gjp.response.rate[0];
    }
    ROS_INFO("pos: %d %f", i, gjp.response.position[0]);
  }

  pidControl();
}


void BaseController::pidControl() {
  for (auto &pc: pidControllers) {
    pc.pidControl();
  }
}
