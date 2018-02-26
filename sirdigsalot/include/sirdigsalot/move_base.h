#ifndef SIRDIGSALOT_MOVE_BASE_H_
#define SIRDIGSALOT_MOVE_BASE_H_

#include <future>
#include <memory>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/tf.h>

#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

struct MoveResult {
public:
  enum Result {
    SUCCESS,
    FAILED,
    TIMEOUT
  };

  MoveResult(Result r);
  operator bool() const;

  Result result;
};

/// This wraps move_base commands into MoveGoals, which can be used to block
/// the high level thread until the operation has been completed
class MoveBase: public Ticker
{
protected:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
public:
  void Init();
  void Tick() override;

  /// Moves the robot into a new pose within the timeout period
  /// Set the timeout to zero to just wait into the target pose is reached
  MoveResult MoveTo(tf::Pose pose, ros::Duration timeout);

  /// Moves the robot in the direction of the point
  /// Set the timeout to zero to wait until the target position is reached
  MoveResult MoveDelta(tf::Point point, ros::Duration timeout);

  /// Creates an instance of MoveBase and registers it to the TickerManager
  static std::shared_ptr<MoveBase> CreateInstance(TickerManager& manager);

protected:
  MoveBase();
  bool inited;

  std::shared_ptr<Client> client;
};

}

#endif
