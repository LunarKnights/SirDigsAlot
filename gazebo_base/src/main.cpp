#include "ros/ros.h"
#include "ros/console.h"

#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/SpawnModel.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "BaseController.h"
#include "PIDController.h"

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_base");

  auto nh = ros::NodeHandle();

  // set up private node handler to get parameters, as per
  // http://answers.ros.org/question/11098/roscpp-relative-parameter/
  auto nhPrivate = ros::NodeHandle("~");

  // wait for gazebo to come up
  {
    ROS_INFO("waiting for gazebo...");
    int timeout_count = 5;
    int timeout_time = 5;
    while (timeout_count > 0)
    {
      if (ros::service::waitForService("/gazebo/spawn_urdf_model", timeout_time))
      {
        break;
      }
      timeout_count--;
      ROS_INFO("/gazebo/spawn_urdf_model connection timed out, retry %d", 5 - timeout_count);
    }
    if (timeout_count <= 0)
    {
      ROS_ERROR("unable to connect to gazebo");
      return -5;
    }
  }

  ROS_INFO("spawning model...");
  // spawn the robot model in gazebo
  if (!SpawnModel(nh, nhPrivate))
  {
    ROS_ERROR("unable to spawn model");
    return -1;
  }

  auto aje_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort", true);
  auto cjf_client = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces", true);
  auto gj_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties", true);

  if (!aje_client)
  {
    ROS_ERROR("unable to connect to /gazebo/apply_joint_effort");
    return -3;
  }
  if (!cjf_client)
  {
    ROS_ERROR("unable to connect to /gazebo/clear_joint_forces");
    return -4;
  }
  if (!gj_client)
  {
    ROS_ERROR("unable to connect to /gazebo/get_joint_properties");
    return -9;
  }

  auto bc = BaseController(aje_client, cjf_client, gj_client);

  auto base_controller_sub = nh.subscribe("cmd_vel", 1, &BaseController::ControllerCallback, &bc);

  auto r = ros::Rate(100);

  while (ros::ok())
  {
    bc.PollJoints();
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  while (ros::ok())
  {
    r.sleep();
  }

  return 0;
}

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate)
{
  // copy the model into a string to pass into the model spawner
  std::string model_path = "";
  std::stringstream model;

  if (!nhPrivate.getParam("model_path", model_path))
  {
    ROS_ERROR("no model path specified");
    ROS_ERROR("path: %s", model_path.c_str());
    return -3;
  }
  ROS_INFO("opening model %s", model_path.c_str());
  {
    auto model_file = std::ifstream(model_path.c_str());
    model << model_file.rdbuf();
  }

  // spawn model
  std::string model_name = "";
  {
    auto model_spawner = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel sm;
    sm.request.model_name = "tesbot";
    sm.request.model_xml = model.str();
    sm.request.robot_namespace = "tesbot";
    // sm.request.initial_pose;

    if (model_spawner.call(sm))
    {
      if (sm.response.success)
      {
        ROS_INFO("robot spawn successful");
        model_name = sm.request.model_name;
      }
      else
      {
        ROS_ERROR("spawn attempt failed");
        ROS_ERROR("error message: %s", sm.response.status_message.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("unable to connect to model spawner");
      return false;
    }
  }
  return true;
}
