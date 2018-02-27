#ifndef SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_
#define SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_broadcaster.h>

namespace sirdigsalot {

/// This publishes the arena frame for tf so that it's possible to specify
/// coordinates in the arena coordinate system instead of the map coordinate
/// system (whose orientation is essentially arbitrary and based on the 
/// starting position)
class ArenaFrameBroadcaster
{
public:
  ArenaFrameBroadcaster(ros::NodeHandle &nh, ros::NodeHandle &nhPrivate);
  ~ArenaFrameBroadcaster();
  // used to set the arena frame using the known locations of a single point
  // in both spaces
  void SetArenaFrame(const geometry_msgs::Pose &mapPoint, const geometry_msgs::Pose &arenaPoint);
protected:
  static void ThreadLoop(ArenaFrameBroadcaster* me);

  // TODO: use a ROS parameter instead
  double broadcastRate = 10.0;
  std::string mapFrame;
  std::string arenaFrame;

  tf2_ros::TransformBroadcaster broadcaster;

  std::atomic<bool> isRunning;
  std::condition_variable inited;
  std::thread frameRunner;
  std::mutex frameLock;
  std::shared_ptr<geometry_msgs::Transform> frame;
};

}

#endif
