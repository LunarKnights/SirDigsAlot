#include <mutex>

#include <sirdigsalot/ticker.h>
#include <sirdigsalot/ticker_manager.h>

namespace sirdigsalot {

TickerManager::TickerManager()
{
  // do nothing
}

void TickerManager::TickAll()
{
  std::lock_guard<std::mutex> guard(lock);
  for (auto &t: tickers)
  {
    t->Tick();
  }
}

void TickerManager::RegisterTicker(std::shared_ptr<Ticker> ticker)
{
  std::lock_guard<std::mutex> guard(lock);
  tickers.push_back(ticker);
}

}
