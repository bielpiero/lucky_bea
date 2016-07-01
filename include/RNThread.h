#ifndef RN_THREAD_H
#define RN_THREAD_H

#include "RNUtils.h"
#include "RNFunPointer.h"

class RNThread{
public:
	RNThread(bool joinable = true);
	RNThread(RNFunPointer* func, bool joinable = true);
	virtual ~RNThread();

	pid_t getPID() const;
	pthread_t getThread() const;

	int lock();
	int tryLock();
	int unlock();

	int create(RNFunPointer* func, bool joinable=true, bool lowerPriority=true);
	int join(void** ret = NULL);
	int detach(void);
	void cancel(void);
	void stop(void);

	void threadStarted();
	bool isThreadStarted() const;

	void threadFinished();
	bool isThreadRunning() const;
	bool hasThreadFinished() const;
	bool getJoinable() const;
	
	void setThreadName(const char* name);
	const char* getThreadName() const;

	RNFunPointer* getFunctionPointer() const;

	enum RNThreadStatus{
		FAILED = 1,
		NO_RESOURCE,
		NO_SUCH_THREAD,
		INVALID,
		JOIN_SELF,
		ALREADY_DETACHED
	};

private:
	static void* run(void* arg);
protected:
	pthread_t thread;
	pthread_mutex_t mutexLocker;
	RNFunPointer* func;

	bool isStarted;
	bool isRunning;
	bool hasFinished;
	bool joinable;

	std::string threadName;

	pid_t processId;
	pid_t parentProcessId;
};

#endif