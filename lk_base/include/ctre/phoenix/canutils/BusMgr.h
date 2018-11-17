#pragma once
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/ErrorCode.h"
#include "TxJob.h"
#include "RxStream.h"
#include "ManualEvent.h"

#include <stdint.h>
#include <map>
#include <vector>
#include <mutex>

namespace ctre {
namespace phoenix {
namespace platform {
namespace canutil {

class BusMgr
{
public:
	static BusMgr & GetInstance();
	~BusMgr();
	void Dispose();

	//---------------- TxFrames ------------------//
	int32_t SendFrame(uint32_t arbID, const uint8_t *data, uint8_t dataSize, int32_t periodMs);
	//---------------- RxFrames ------------------//
	int32_t ReceiveLastFrame(uint32_t arbID, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp);
	
	int32_t OpenStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages);

	void CloseStreamSession(uint32_t sessionHandle);

	int32_t ReadStreamSession(uint32_t sessionHandle, ctre::phoenix::platform::can::canframe_t *messages, uint32_t messagesToRead, uint32_t *messagesRead);
	
    void StartThreads();

private:

	BusMgr();

	//---------------- Private routines  ------------------//
	/**
	* @return localy unique IDs for stream management.
	*/
	uint32_t GetAllocStreamhandle();
	void BackgroundRx();
	void BackgroundTx();

	//---------------- Private routines  ------------------//
	void StopThreads();
	bool IsRunning();
	static void Background_s_Rx(BusMgr * me);
	static void Background_s_Tx(BusMgr * me);
	//------------- Member variables --------------------//
	typedef std::map<uint32_t, TxJob> mapTxJobs_t;
	mapTxJobs_t _mapTxJobs;
	std::recursive_timed_mutex _lckTxJobs;
	
	typedef std::map<uint32_t, ctre::phoenix::platform::can::canframe_t> mapRxFrames_t;
	mapRxFrames_t _mapRxFrames;
	std::recursive_timed_mutex _lckRxFrames;

	/**
	* Key value is an ordinal that starts at a nonzero value.
	*/
	typedef std::map<uint32_t, RxStream *> mapStreams_t;
	mapStreams_t _mapStreams;
	std::recursive_timed_mutex _lckStreams;

	std::thread * _threadRx = nullptr;
	std::thread * _threadTx = nullptr;
	ManualEvent _isTerminating;
	ManualEvent _isFinishedRx;
	ManualEvent _isFinishedTx;

	/* rolling counter starting at 900 */
	uint32_t _rollingStreamHandle = 900; // Zebracorns

	//---------------- Private routines  ------------------//
	static BusMgr * _instance;
};

} //namespace canutil
} //namespace platform
} //namespace phoenix
} //namespace ctre
