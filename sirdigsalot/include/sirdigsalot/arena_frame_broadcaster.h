#ifndef SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_
#define SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_

#include <memory>
#include <mutex>
#include <thread>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>

namespace sirdigsalot {

/// This publishes the arena frame for tf so that it's possible to specify
/// coordinates in the arena coordinate system instead of the map coordinate
/// system (whose orientation is essentially arbitrary and based on the 
/// starting position)
class ArenaFrameBroadcaster
{
public:
  ArenaFrameBroadcaster();
  ~ArenaFrameBroadcaster();
  // used to set the arena frame using the known locations of a single point
  // in both spaces
  void SetArenaFrame(tf::Point mapPoint, tf::Point arenaPoint);
protected:
  static void ThreadLoop(ArenaFrameBroadcaster* me);
  std::thread frameRunner;
  std::mutex frameLock;
  std::shared_ptr<tf::Transform> frame;
};

}

#endif
