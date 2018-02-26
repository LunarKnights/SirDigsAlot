#ifndef SIRDIGSALOT_TICKER_MANAGER_H_
#define SIRDIGSALOT_TICKER_MANAGER_H_

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace sirdigsalot {

class TickerManager {
public:
  TickerManager();
  void TickAll();
  void RegisterTicker(std::shared_ptr<Ticker> ticker);

protected:
  std::mutex lock;
  std::vector<std::shared_ptr<Ticker>> tickers;
};

}

#endif
