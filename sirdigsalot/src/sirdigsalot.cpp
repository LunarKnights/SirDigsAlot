/// This is the main file for the lk_rover node
/// This is the main logic for the system
/// Below we have a lot of #includes for:

/// The basic #includes you need when you're writing ROS code;
/// these provide functions that you need to talk to other ROS nodes
#include "ros/ros.h"
#include "ros/console.h"

#include <sirdigsalot/movement_manager.h>
#include <sirdigsalot/ticker_manager.h>

using namespace sirdigsalot;

int main(int argc, char** argv)
{
  /// At the top of any ROS program, you'll probably need to call ros::init
  ros::init(argc, argv, "sirdigsalot");

  /// ... and then this stuff to get access to ROS topic stuff
  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  auto r = ros::Rate(100);

  TickerManager tickerManager;
  auto movementManager = MovementManager::CreateInstance(tickerManager);

  std::thread([=]() {
    // TODO: high level logic here
  });

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    // run any low-level logic stuff here
    tickerManager.TickAll();
  }
  ros::spin();

  return 0;
}

