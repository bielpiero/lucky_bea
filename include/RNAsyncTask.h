#ifndef RN_ASYNC_TASK_H
#define RN_ASYNC_TASK_H

#include "RNThread.h"

class RNAsyncTask : public RNThread{
public:
	RNAsyncTask();
	virtual ~RNAsyncTask();
	virtual void* runThread(void* arg) = 0;
	virtual void run(void);
	virtual void* runInThisThread(void* arg = 0);

	virtual void runAsync(void) { create(); }

	virtual int create(bool joinable = true, bool lowerPriority = true);
private:
	RNRetFunPointerClass1<void*, RNAsyncTask, void*>* func;
};

#endif