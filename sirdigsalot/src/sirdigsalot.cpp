/// This is the main file for the lk_rover node
/// This is the main logic for the system
/// Below we have a lot of #includes for:

/// The basic #includes you need when you're writing ROS code;
/// these provide functions that you need to talk to other ROS nodes
#include "ros/ros.h"
#include "ros/console.h"

int main(int argc, char** argv)
{
  /// At the top of any ROS program, you'll probably need to call ros::init
  ros::init(argc, argv, "sirdigsalot");

  /// ... and then this stuff to get access to ROS topic stuff
  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  auto r = ros::Rate(100);

  while (ros::ok())
  {
    r.sleep();
    // TODO: Put high level planning stuff here
  }

  return 0;
}
