#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

std::atomic<bool> teleopDetected(false);

// used to let multiplexers check to see if they've timed out
class Timeout
{
public:
  // return true if a message hasn't been received within the timeout period
  virtual bool CheckTimeout() = 0;
};

ros::Publisher killMotors;

std::mutex timeoutsLock;
std::vector<Timeout*> timeouts;
std::atomic<bool> killed(false);

void timerCb(const ros::TimerEvent& event)
{
  std::lock_guard<std::mutex> timeoutGuard(timeoutsLock);
  bool timeout = false;
  for (auto t: timeouts)
  {
    if (t && t->CheckTimeout())
    {
      timeout = true;
      break;
    }
  }
  if (timeout)
  {
    ROS_WARN("timeout conditions reached!");
    std_msgs::Bool msg;
    msg.data = true;

    killed = true;
    killMotors.publish(msg);
  }
}


template <typename T> class Multiplexer: public Timeout
{
public:
  Multiplexer(ros::NodeHandle &nh,
      const std::string &pubTopic,
      const std::string &teleopTopic,
      const std::string &autoTopic,
      const ros::Duration &timeout=ros::Duration(0.5)): timeout(timeout)
  {
    ROS_INFO("mux: %s, %s -> %s", teleopTopic.c_str(), autoTopic.c_str(), pubTopic.c_str());
    pub = nh.advertise<T>(pubTopic, 1);
    teleop = nh.subscribe(teleopTopic, 1, &Multiplexer<T>::teleopCb, this);
    auton = nh.subscribe(teleopTopic, 1, &Multiplexer<T>::autonCb, this);
    {
      std::lock_guard<std::mutex> timeoutGuard(timeoutsLock);
      timeouts.push_back(this);
    }
  }

  ~Multiplexer()
  {
      std::lock_guard<std::mutex> timeoutGuard(timeoutsLock);
      auto it = std::find(timeouts.begin(), timeouts.end(), this);
      if (it != timeouts.end())
      {
        timeouts.erase(it);
      }
  }

  void teleopCb(const T& msg)
  {
    {
      std::lock_guard<std::mutex> guard(lastTimeLock);
      lastTime = ros::Time::now();
    }
    if (!teleopDetected)
    {
      ROS_INFO("teleoperation detected!");
      teleopDetected = true;
    }
    if (killed)
    {
      killed = false;

      std_msgs::Bool msg;
      msg.data = false;
      killMotors.publish(msg);
    }
    pub.publish(msg);
  }

  void autonCb(const T& msg)
  {
    {
      std::lock_guard<std::mutex> guard(lastTimeLock);
      lastTime = ros::Time::now();
    }
    if (!teleopDetected)
    {
      if (killed)
      {
        killed = false;

        std_msgs::Bool msg;
        msg.data = false;
        killMotors.publish(msg);
      }

      pub.publish(msg);
    }
  }

  bool CheckTimeout() override
  {
    std::lock_guard<std::mutex> guard(lastTimeLock);
    return (ros::Time::now() - lastTime) > timeout;
  }
protected:
  ros::Publisher pub;
  ros::Subscriber teleop, auton;

  std::mutex lastTimeLock;
  ros::Time lastTime;
  ros::Duration timeout;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lk_control_mux");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  // TODO: set up timer callback

  auto r = ros::Rate(100);

  // TODO: add more multiplexers for all the actuators
  std::string killMotorsTopic;
  std::string cmdVelOutput, cmdVelTele, cmdVelAuto;

  nhPrivate.param<std::string>("kill_topic", killMotorsTopic, "kill_motors");
  nhPrivate.param<std::string>("cmd_vel_output", cmdVelOutput, "cmd_vel");
  nhPrivate.param<std::string>("cmd_vel_tele", cmdVelTele, "tele_cmd_vel");
  nhPrivate.param<std::string>("cmd_vel_auto", cmdVelAuto, "auto_cmd_vel");

  ROS_INFO("kill motors topic: %s", killMotorsTopic.c_str());
  auto timeoutTimer = nh.createTimer(ros::Duration(0.5), timerCb);
  killMotors = nh.advertise<std_msgs::Bool>(killMotorsTopic, 1);

  Multiplexer<geometry_msgs::Twist>(nh, cmdVelOutput, cmdVelTele, cmdVelAuto);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
