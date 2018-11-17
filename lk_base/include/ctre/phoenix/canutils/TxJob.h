#pragma once
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/ErrorCode.h"

#include <stdint.h>
#include <map>
#include <vector>
#include <mutex>

namespace ctre {
namespace phoenix {
namespace platform {
namespace canutil {

class TxJob {
public:
	ctre::phoenix::platform::can::canframe_t frame;
	std::chrono::time_point<std::chrono::steady_clock> timer;
	uint32_t period;

	TxJob();
	TxJob(const TxJob & rhs);
	TxJob & operator = (const TxJob & rhs);
	bool operator < (const TxJob & rhs);
};

} //namespace canutil
} //namespace platform
} //namespace phoenix
} //namespace ctre
