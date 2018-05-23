#include "RNAsyncTask.h"

RNAsyncTask::RNAsyncTask(){
	func = new RNRetFunPointerClass1<void* ,RNAsyncTask, void*>(this, &RNAsyncTask::runThread, NULL);
}

RNAsyncTask::~RNAsyncTask(){
	
}

void* RNAsyncTask::runInThisThread(void* arg){
	joinable = true;
	isRunning = true;
	if(this->threadName.size() == 0){
		RNUtils::printLn("Running anonymous thread with id %llu", this->thread);
	} else {
		RNUtils::printLn("Running thread %s with Id %llu", this->threadName.c_str(), this->thread);
	} 

	return runThread(arg);
}

int RNAsyncTask::create(bool joinable, bool lowerPriority){
	return RNThread::create(func, joinable, lowerPriority);
}

void RNAsyncTask::run(void){ 
	this->runInThisThread(); 
}