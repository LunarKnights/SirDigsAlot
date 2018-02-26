#ifndef SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_
#define SIRDIGSALOT_ARENA_FRAME_BROADCASTER_H_

#include <future>
#include <memory>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <sirdigsalot/ticker.h>

namespace sirdigsalot {

/// This publishes the arena frame for tf so that it's possible to specify
/// coordinates in the arena coordinate system instead of the map coordinate
/// system (whose orientation is essentially arbitrary and based on the 
/// starting position)
class ArenaFrameBroadcaster
{
public:
  // used to set the arena frame using the known locations of a single point
  // in both spaces
  ArenaFrameBroadcaster(); // TODO
  ~ArenaFrameBroadcaster(); // TODO
  void SetArenaFrame(tf::Point mapPoint, tf::Point arenaPoint); // TODO
protected:
  std::thread frameRunner;
  std::mutex frameLock;
  std::shared_ptr<tf::Transform> frame;
};

}

#endif
