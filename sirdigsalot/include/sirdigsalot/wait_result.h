#ifndef SIRDIGSALOT_WAIT_RESULT_H_
#define SIRDIGSALOT_WAIT_RESULT_H_

struct WaitResult {
public:
  enum Result {
    SUCCESS,
    FAILED,
    TIMEOUT
  };

  WaitResult(Result r);
  operator bool() const;

  Result result;
};


#endif
