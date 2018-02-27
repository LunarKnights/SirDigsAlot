#include <sirdigsalot/wait_result.h>

WaitResult::WaitResult(WaitResult::Result r) : result(r) 
{
  // do nothing
}

WaitResult::operator bool() const
{
  return result == SUCCESS;
}

WaitResult WaitResult::FromGoalState(const actionlib::SimpleClientGoalState &state)
{
  switch (state.state_) {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      return WaitResult(WaitResult::SUCCESS);
    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_WARN("goal rejected!");
      return WaitResult(WaitResult::FAILED);
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_WARN("goal aborted!");
      return WaitResult(WaitResult::FAILED);
    case actionlib::SimpleClientGoalState::LOST:
      ROS_WARN("goal lost!");
      return WaitResult(WaitResult::FAILED);
    default:
      ROS_ERROR("goal returned unexpected state!");
      return WaitResult(WaitResult::FAILED);
  }
  ROS_ERROR("unreachable code!");
  return WaitResult(WaitResult::FAILED);
}
