#include <mutex>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sirdigsalot/arena_frame_broadcaster.h>

namespace sirdigsalot {

ArenaFrameBroadcaster::ArenaFrameBroadcaster(ros::NodeHandle &nh, ros::NodeHandle &nhPrivate): isRunning(true)
{
  nh.param<std::string>("map_frame_id", mapFrame, "map");
  nh.param<std::string>("arena_frame_id", arenaFrame, "arena");

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
  const bool hadFrame = static_cast<bool>(frame);
  if (!hadFrame)
  {
    frame = std::make_shared<geometry_msgs::Transform>();
  }

  tf2::Transform mapPose_, arenaPose_;
  tf2::fromMsg(mapPoint, mapPose_);
  tf2::fromMsg(arenaPoint, arenaPose_);

  const auto mapPose = mapPose_, arenaPose = arenaPose_;

  // http://wiki.ros.org/tf/Overview/Transformations
  // find the transform between the poses
  // TODO: make sure this is correct
  const auto transform = mapPose * arenaPose.inverse();
  *frame = tf2::toMsg(transform);
  // notify the thread loop if this is the first frame
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
  ROS_INFO("first frame detected, broadcasting transform...");
  auto r = ros::Rate(me->broadcastRate);
  while (me->isRunning)
  {
    {
      std::lock_guard<std::mutex> guard(me->frameLock);
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = me->mapFrame;
      transformStamped.child_frame_id = me->arenaFrame;
      transformStamped.transform = *(me->frame);

      me->broadcaster.sendTransform(transformStamped);
    }
    r.sleep();
  }
  ROS_INFO("arena broadcaster exiting...");
}

}
