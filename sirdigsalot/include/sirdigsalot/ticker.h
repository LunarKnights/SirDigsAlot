#ifndef SIRDIGSALOT_TICKER_H_
#define SIRDIGSALOT_TICKER_H_

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace sirdigsalot {

// Used as a base class for all the objects that need
// to be called frequently to update stuff
// This will be called once per ROS spin loop cycle, which mean it's called around
// once per ~10ms
class Ticker
{
public:
  virtual void Tick() = 0;
};

}

#endif
