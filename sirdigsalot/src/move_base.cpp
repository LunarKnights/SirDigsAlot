#include <mutex>

#include <sirdigsalot/move_base.h>
#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

MoveBase::MoveBase():
    client(std::shared_ptr<Client>(new Client("move_base", true))),
    inited(false)
{
  // do nothing
}

std::shared_ptr<MoveBase> MoveBase::CreateInstance(TickerManager& manager)
{
  auto instance = std::shared_ptr<MoveBase>(new MoveBase());
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

MoveResult MoveBase::MoveTo(tf::Pose pose, ros::Duration timeout)
{
  // TODO
}

MoveResult MoveBase::MoveDelta(tf::Point point, ros::Duration timeout)
{
  // TODO
}

MoveResult::MoveResult(MoveResult::Result r) : result(r) 
{
  // do nothing
}

MoveResult::operator bool() const
{
  return result == SUCCESS;
}

}
