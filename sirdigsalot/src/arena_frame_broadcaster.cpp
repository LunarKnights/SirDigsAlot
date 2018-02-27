#include <lock_guard>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sirdigsalot/arena_frame_broadcaster.h>

namespace sirdigsalot {

ArenaFrameBroadcaster::ArenaFrameBroadcaster()
{
  isRunning = true;
  frameRunner = std::thread(ArenaFrameBroadcaster::ThreadLoop, this);
}

ArenaFrameBroadcaster::~ArenaFrameBroadcaster()
{
  isRunning = false;
  // make sure to unblock the thread
  inited.notify_one();
  ROS_INFO("waiting for arena frame broadcaster to die...");
  frameRunner.join();
}

void ArenaFrameBroadcaster::SetArenaFrame(const geometry_msgs::Pose &mapPoint, const geometry_msgs::Pose &arenaPoint)
{
  std::lock_guard<std::mutex> guard(frameLock);
  bool hadFrame = frame;
  if (!hadFrame)
  {
    frame = std::make_shared<geometry_msgs::Transform>();
  }

  tf2::Transform mapPose_, arenaPose_;
  tf2::fromMsg(mapPoint, mapPose_);
  tf2::fromMsg(arenaPoint, arenaPose_);

  const auto mapPose = mapPose_, arenaPose = arenaPose_;

  // http://wiki.ros.org/tf/Overview/Transformations
  *frame
  // TODO: find the transform between the poses
  if (!hadFrame)
  {
    inited.notify_one();
  }
}

void ArenaFrameBroadcaster::ThreadLoop(ArenaFrameBroadcaster* me)
{
  {
    while (me->isRunning)
    {
      std::unique_lock<std::mutex> ul(me->frameLock);
      me->inited.wait(ul);
      if (me->frame)
      {
        break;
      }
    }
  }
  while (me->isRunning)
  {
    {
      std::lock_guard<std::mutex> guard(me->frameLock);
      // TODO
    }
    // TODO sleep
  }
}

}
