#ifndef RN_TIMER_H
#define RN_TIMER_H

#include "RNAsyncTask.h"
#include <chrono>

class RNTimer : public RNAsyncTask{
public:
	RNTimer(const char* name = "", const char* description = "");
	virtual ~RNTimer();
	
	void start();
	void stop();
	void setInterval(long long int msecs);
	void reset();

	void addTimeoutCB(RNFunPointer* timeoutCB);
private:
	void* runThread(void* object);
	void task();
private:
	std::string name;
	std::string description;
	bool running;
	bool goRequested;
	bool killed;

	RNFunPointer* timeoutCB;
	std::chrono::milliseconds msecs;
	std::chrono::high_resolution_clock::time_point begin;
};

#endif