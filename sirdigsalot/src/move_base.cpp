#include <mutex>

#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sirdigsalot/move_base.h>
#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

MoveBase::MoveBase(ros::NodeHandle &nh, ros::NodeHandle &nhPriv):
    nh(nh),
    nhPriv(nhPriv),
    inited(false),
    tfBuffer(),
    tfListener(tfBuffer)
{
  // TODO: use rosparams to parameterize move_base topic
  client = std::shared_ptr<Client>(new Client("move_base", true));
  // TODO: use rosparams to get map frame
  mapFrame = "map";
  // TODO: use rosparams to get arena frame
  arenaFrame = "arena";
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

WaitResult MoveBase::MoveTo(double x, double y, double angle, ros::Duration timeout)
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
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = now;
  goal.target_pose.pose = mapPose;

  auto ret = client->sendGoalAndWait(goal, timeout);

  // process the returned value
  switch (ret.state_) {
    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_WARN("move_base goal rejected!");
      return WaitResult(WaitResult::FAILED);
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_WARN("move_base goal aborted!");
      return WaitResult(WaitResult::FAILED);
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      return WaitResult(WaitResult::SUCCESS);
    case actionlib::SimpleClientGoalState::LOST:
      ROS_WARN("move_base goal lost!");
      return WaitResult(WaitResult::FAILED);
    default:
      ROS_ERROR("move_base returned unexpected state!");
      return WaitResult(WaitResult::FAILED);
  }
  ROS_ERROR("unreachable code!");
  return WaitResult(WaitResult::FAILED);
}

WaitResult MoveBase::MoveDelta(double x, double y, ros::Duration timeout)
{
  // TODO
  return WaitResult(WaitResult::FAILED);
}

WaitResult MoveBase::SetMaxVelocity(double vel)
{
  // TODO
  return WaitResult(WaitResult::FAILED);
}

WaitResult MoveBase::SetMinVelocity(double vel)
{
  // TODO
  return WaitResult(WaitResult::FAILED);
}

}
