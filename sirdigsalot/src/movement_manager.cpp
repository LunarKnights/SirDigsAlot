#include <mutex>

#include <sirdigsalot/movement_manager.h>
#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

MovementManager::MovementManager():
    client("move_base", true),
    events(),
    inited(false)
{
}

std::shared_ptr<MovementManager> MovementManager::CreateInstance(TickerManager& manager)
{
  auto instance = std::shared_ptr<MovementManager>(new MovementManager());
  manager.RegisterTicker(std::static_pointer_cast<Ticker>(instance));
  return instance;
}

void MovementManager::Init()
{
  if (!inited)
  {
    ROS_INFO("waiting for move_base actionlib server...");
    client.waitForServer();
    ROS_INFO("server detected!");
  }
}

void MovementManager::Tick()
{
  std::lock_guard<std::mutex> guard(lock);
  // TODO: check goals
}

}
