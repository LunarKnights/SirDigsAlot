#pragma once

#include <condition_variable>
#include <thread>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <chrono>

namespace ctre {
namespace phoenix {
namespace platform {
namespace canutil {

/**
* Linux equivalent to Window's Manual Event object for thread concurrency.
* @author Ozrien
*/
class ManualEvent {
public:
	/**
	* Constructor
	* @param initialSignal object defaults to Signaled or Not Signaled, (default is not signaled).
	*/
	ManualEvent(bool initialSignal = false)
	{
		_signal = initialSignal;
	}

	/** D'tor */
	~ManualEvent()
	{
		/* MT */
	}

	/**
	*  wait for event to be signaled. or for time out
	*  @return true if event is signaled, false if timed out.
	*/
	bool WaitForSignal(int timeoutMs)
	{

		std::unique_lock < std::mutex > lk(_m);

		std::chrono::milliseconds ms(timeoutMs);

		/* if signal is already set, don't bother checking cond variable */
		if (_signal)
			return true;

		_cv.wait_for(lk, ms);

		/* was the signal event a set? */
		if (_signal)
			return true;

		/* Notify caller that we timed out.
		*
		* this is naive as it assumes any signal changes are Sets and not Clears.
		* We should add a time tracker and re-attempt wait_for if signal is still false.
		* Otherwise we may time out before timeoutMs occurs.
		*
		* If we are just doing one-shot events, this is fine.
		*/
		return false;
	}

	/**
	*  Signal the event.  Any threads waiting on WaitForSignal() will return true.
	*/
	void Signal()
	{
		/* changed signal within critical section */
		std::unique_lock < std::mutex > lk(_m);
		_signal = true;
		lk.unlock();

		/* signal any threads waiting on an update */
		_cv.notify_all();
	}
	/**
	*  Clear the event.  Any threads waiting on WaitForSignal() will time out.
	*/
	void Clear()
	{
		/* changed signal within critical section */
		std::unique_lock < std::mutex > lk(_m);
		_signal = false;
		lk.unlock();

		/* signal any threads waiting on an update */
		_cv.notify_all();
	}
private:

	bool _signal = false;
	std::mutex _m;
	std::condition_variable _cv;

	/** hidden copy c'tor */
	ManualEvent(const ManualEvent &) {}
};

} //namespace canutil
} //namespace platform
} //namespace phoenix
} //namespace ctre
