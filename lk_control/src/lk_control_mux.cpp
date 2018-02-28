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

  // TODO: get values from params
  std::string cmdVelOutput = "cmd_vel";
  std::string cmdVelTele = "tele_cmd_vel";
  std::string cmdVelAuto = "auto_cmd_vel";

  auto r = ros::Rate(100);

  Multiplexer<geometry_msgs::Twist>(nh, cmdVelOutput, cmdVelTele, cmdVelAuto);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
