#include <atomic>
#include <string>

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"

std::atomic<bool> teleopDetected(false);

// TODO: add timeout safety stuff
template <typename T> class Multiplexer
{
public:
  Multiplexer(ros::NodeHandle &nh,
      const std::string &pubTopic,
      const std::string &teleopTopic,
      const std::string &autoTopic)
  {
    ROS_INFO("mux: %s, %s -> %s", teleopTopic.c_str(), autoTopic.c_str(), pubTopic.c_str());
    pub = nh.advertise<T>(pubTopic, 1);
    teleop = nh.subscribe(teleopTopic, 1, &Multiplexer<T>::teleopCb, this);
    auton = nh.subscribe(teleopTopic, 1, &Multiplexer<T>::autonCb, this);
  }

  void teleopCb(const T& msg)
  {
    if (!teleopDetected)
    {
      ROS_INFO("teleoperation detected!");
      teleopDetected = true;
    }
    pub.publish(msg);
  }

  void autonCb(const T& msg)
  {
    if (!teleopDetected)
    {
      pub.publish(msg);
    }
  }
protected:
  ros::Publisher pub;
  ros::Subscriber teleop, auton;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lk_control_mux");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  auto r = ros::Rate(100);

  // TODO: add more multiplexers for all the actuators
  std::string cmdVelOutput, cmdVelTele, cmdVelAuto;

  nhPrivate.param<std::string>("cmd_vel_output", cmdVelOutput, "cmd_vel");
  nhPrivate.param<std::string>("cmd_vel_tele", cmdVelTele, "tele_cmd_vel");
  nhPrivate.param<std::string>("cmd_vel_auto", cmdVelAuto, "auto_cmd_vel");

  Multiplexer<geometry_msgs::Twist>(nh, cmdVelOutput, cmdVelTele, cmdVelAuto);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
