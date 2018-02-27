#ifndef SIRDIGSALOT_MOVE_BASE_H_
#define SIRDIGSALOT_MOVE_BASE_H_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>
#include <sirdigsalot/wait_result.h>

namespace sirdigsalot {

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
  WaitResult MoveTo(double x, double y, double angle, ros::Duration timeout);

  /// Moves the robot in the direction of the point
  /// Set the timeout to zero to wait until the target position is reached
  WaitResult MoveDelta(double x, double y, ros::Duration timeout);

  /// Sets move_base's base_local_planner max_vel_x parameter to allow the robot's speed
  /// to be adjusted as needed
  WaitResult SetMaxVelocity(double vel);

  /// Sets move_base's base_local_planner min_vel_x parameter to allow the robot's speed
  /// to be adjusted as needed
  WaitResult SetMinVelocity(double vel);

  /// Creates an instance of MoveBase and registers it to the TickerManager
  static std::shared_ptr<MoveBase> CreateInstance(ros::NodeHandle &nh, ros::NodeHandle &nhPriv, TickerManager& manager);
protected:
  MoveBase(ros::NodeHandle &nh, ros::NodeHandle &nhPriv);

  std::string mapFrame, arenaFrame;

  ros::NodeHandle nh, nhPriv;
  bool inited;
  tf2::BufferCore tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::shared_ptr<Client> client;
};

}

#endif
