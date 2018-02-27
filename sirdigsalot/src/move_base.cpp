#include <mutex>

#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sirdigsalot/move_base.h>
#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

MoveBase::MoveBase(ros::NodeHandle &nh, ros::NodeHandle &nhPriv):
    inited(false),
    tfBuffer(),
    tfListener(tfBuffer)
{
  // NOTE: need to make sure the right node is being used here
  nh.param<std::string>("move_base_topic", moveBaseTopic, "move_base");
  nh.param<std::string>("map_frame_id", mapFrame, "map");
  nh.param<std::string>("arena_frame_id", arenaFrame, "arena");
  nh.param<std::string>("base_frame_id", baseFrame, "base_link");

  client = std::shared_ptr<Client>(new Client(moveBaseTopic, true));
}

std::shared_ptr<MoveBase> MoveBase::CreateInstance(ros::NodeHandle &nh, ros::NodeHandle &nhPriv, TickerManager& manager)
{
  auto instance = std::shared_ptr<MoveBase>(new MoveBase(nh, nhPriv));
  manager.RegisterTicker(std::static_pointer_cast<Ticker>(instance));
  return instance;
}

void MoveBase::Init()
{
  if (!inited)
  {
    ROS_INFO("waiting for move_base actionlib server...");
    client->waitForServer();
    ROS_INFO("server detected!");
  }
}

void MoveBase::Tick()
{
  // do nothing
  // NOTE: maybe later this will need to do something
}

WaitResult MoveBase::MoveTo(const double x, const double y, const double angle, const ros::Duration& timeout)
{
  const auto now = ros::Time::now();
  // look up arena frame to map frame transform; fail quickly if not found
  if (!tfBuffer.canTransform(mapFrame, arenaFrame, now))
  {
    ROS_ERROR("unable to get arena to map transform!");
    return WaitResult(WaitResult::FAILED);
  }

  // actually look up arena frame to map frame transform
  const auto stampedTransform = tfBuffer.lookupTransform(mapFrame, arenaFrame, now);
  
  geometry_msgs::Pose arenaPose_;

  arenaPose_.position.x = x;
  arenaPose_.position.y = y;
  arenaPose_.position.z = 0.0;
  auto targetAngle = tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), angle);
  arenaPose_.orientation = tf2::toMsg(targetAngle);

  const auto arenaPose = arenaPose_;

  // transform pose into the map frame
  geometry_msgs::Pose mapPose;
  // hand copying the code here because this doesn't work for some reason
  // tf2::doTransform(arenaPose, mapPose, stampedTransform);
  {
    tf2::Vector3 v;
    tf2::fromMsg(arenaPose.position, v);
    tf2::Quaternion r;
    tf2::fromMsg(arenaPose.orientation, r);

    tf2::Transform t;
    tf2::fromMsg(stampedTransform.transform, t);
    tf2::Transform v_out = t * tf2::Transform(r, v);
    tf2::toMsg(v_out, mapPose);
  }

  // fill in a goal for move_base and send it out
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = baseFrame;
  goal.target_pose.header.stamp = now;
  goal.target_pose.pose = mapPose;

  auto ret = client->sendGoalAndWait(goal, timeout);

  // process the returned value
  return WaitResult::FromGoalState(ret);
}

WaitResult MoveBase::MoveForward(const double x, const ros::Duration &timeout)
{
  const auto now = ros::Time::now();
  // look up base frame to map frame transform; fail quickly if not found
  if (!tfBuffer.canTransform(mapFrame, baseFrame, now))
  {
    ROS_ERROR("unable to get base link to map transform!");
    return WaitResult(WaitResult::FAILED);
  }

  // actually look up arena frame to map frame transform
  const auto stampedTransform = tfBuffer.lookupTransform(mapFrame, baseFrame, now);
  
  geometry_msgs::Pose arenaPose_;

  // REP 103 recommends having the x axis of the base link frame matching the
  // longitudinal axis of the robot:
  // http://www.ros.org/reps/rep-0103.html
  arenaPose_.position.x = x;
  arenaPose_.position.y = 0.0;
  arenaPose_.position.z = 0.0;
  auto targetAngle = tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 0.0);
  arenaPose_.orientation = tf2::toMsg(targetAngle);

  const auto arenaPose = arenaPose_;

  // transform pose into the map frame
  geometry_msgs::Pose mapPose;
  // hand copying the code here because this doesn't work for some reason
  // tf2::doTransform(arenaPose, mapPose, stampedTransform);
  {
    tf2::Vector3 v;
    tf2::fromMsg(arenaPose.position, v);
    tf2::Quaternion r;
    tf2::fromMsg(arenaPose.orientation, r);

    tf2::Transform t;
    tf2::fromMsg(stampedTransform.transform, t);
    tf2::Transform v_out = t * tf2::Transform(r, v);
    tf2::toMsg(v_out, mapPose);
  }

  // fill in a goal for move_base and send it out
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = baseFrame;
  goal.target_pose.header.stamp = now;
  goal.target_pose.pose = mapPose;

  auto ret = client->sendGoalAndWait(goal, timeout);

  // process the returned value
  return WaitResult::FromGoalState(ret);
}

WaitResult MoveBase::SetMaxVelocity(const double vel)
{
  // TODO
  ROS_ERROR("unimplemented!");
  return WaitResult(WaitResult::FAILED);
}

WaitResult MoveBase::SetMinVelocity(const double vel)
{
  // TODO
  ROS_ERROR("unimplemented!");
  return WaitResult(WaitResult::FAILED);
}

}
