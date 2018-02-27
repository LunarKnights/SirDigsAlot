#include <sirdigsalot/wait_result.h>

WaitResult::WaitResult(WaitResult::Result r) : result(r) 
{
  // do nothing
}

WaitResult::operator bool() const
{
  return result == SUCCESS;
}


