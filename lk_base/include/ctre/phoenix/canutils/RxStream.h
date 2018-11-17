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

class RxStream {
public:
	RxStream(uint32_t capacity, uint32_t arbID_Value, uint32_t arbID_Mask);
	~RxStream();
	ctre::phoenix::ErrorCode ProcessFrame(const ctre::phoenix::platform::can::canframe_t & frame);
	int Count();
	const ctre::phoenix::platform::can::canframe_t & Front();
	void Pop();
	bool HasOverflowed();
private:
	void Insert(const ctre::phoenix::platform::can::canframe_t & frame);

	ctre::phoenix::platform::can::canframe_t * _buf;
	uint32_t _capacity;
	uint32_t _arbID_Value;
	uint32_t _arbID_Mask;

	uint32_t _in = 0;
	uint32_t _ou = 0;
	uint32_t _cnt = 0;
	uint32_t _flags = 0;

	RxStream(const RxStream &);
	RxStream operator = (const RxStream &);

	std::recursive_timed_mutex _lck;
};

} //namespace canutil
} //namespace platform
} //namespace phoenix
} //namespace ctre

