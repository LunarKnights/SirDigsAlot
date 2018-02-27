#ifndef SIRDIGSALOT_WAIT_RESULT_H_
#define SIRDIGSALOT_WAIT_RESULT_H_

#include <actionlib/client/simple_action_client.h>

struct WaitResult {
public:
  enum Result {
    SUCCESS,
    FAILED,
    TIMEOUT
  };

  WaitResult(Result r);
  static WaitResult FromGoalState(const actionlib::SimpleClientGoalState &state);
  operator bool() const;

  Result result;
};


#endif
