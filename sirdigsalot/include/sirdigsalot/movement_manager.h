#ifndef SIRDIGSALOT_MOVEMENT_H_
#define SIRDIGSALOT_MOVEMENT_H_

#include <future>
#include <memory>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

/// This wraps move_base commands into MoveGoals, which can be used to block
/// the high level thread until the operation has been completed
class MovementManager: public Ticker
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
protected:
  class MoveGoal
  {
  public:
      enum Result {
        SUCCESS,
        TIMEOUT,
        FAILED
      };
  public:
    Result Wait();
    Result Wait(ros::Duration timeout);
  protected:
  };


public:
  void Init();
  void Tick() override;
  // TODO: add a thread that will broadcast the arena frame transform to allow
  // easy movement within this frame

  // TODO: add function to set the arena frame when the robot's orientation is
  // detected

  MoveGoal MoveTo(double x, double y, double angle);
  MoveGoal MoveDelta(double x, double y, double angle);

  static std::shared_ptr<MovementManager> CreateInstance(TickerManager& manager);

protected:
  MovementManager();
  struct Event {
    // TODO
  };
  std::mutex lock;

  bool inited;
  std::vector<Event> events;

  Client client;
};

}

#endif
