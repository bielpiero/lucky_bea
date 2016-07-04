#include "RNThread.h"

RNThread::RNThread(bool joinable){
	this->func = NULL;
	this->joinable = joinable;
	this->isStarted = false;
	this->isRunning = false;
	this->hasFinished = false;

	this->processId = 0;
	this->parentProcessId = 0;

	this->threadName = std::string("");

	pthread_mutex_init(&mutexLocker, NULL);
}

RNThread::RNThread(RNFunPointer* func, bool joinable){
	this->func = func;

	this->joinable = joinable;
	this->isStarted = false;
	this->isRunning = false;
	this->hasFinished = false;

	this->processId = 0;
	this->parentProcessId = 0;

	this->threadName = std::string("");

	pthread_mutex_init(&mutexLocker, NULL);
	create(func, joinable);
}

int RNThread::create(RNFunPointer* func, bool joinable, bool lowerPriority){
	this->func = func;
	this->joinable = joinable;
	this->isRunning = true;
	int result;
 	pthread_attr_t attr;
 	pthread_attr_init(&attr);
 	if (joinable){
 		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
 	} else {
 		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
 	}

 	if((result = pthread_create(&this->thread, &attr, run, (void*)this)) != 0){
 		pthread_attr_destroy(&attr);
 		switch (result){
 			case EAGAIN:
				RNUtils::printLn("RNThread::create: Error in create: Not enough system resources in pthread_create() (EAGAIN)");
				result = NO_RESOURCE;
				break;
			case ENOMEM:
				RNUtils::printLn("RNThread::create: Error in create: Not enough system resources in pthread_create() (ENOMEM)");
				result = NO_RESOURCE;
				break;
			default:
				RNUtils::printLn("RNThread::create: Error in create: Unknown");
				result = FAILED;
				break;
 		}
 	} else {
 		if(this->threadName.size() == 0){
			RNUtils::printLn("Created anonymous thread with id %ld", this->thread);
		} else {
			RNUtils::printLn("Created thread %s with Id %ld", this->threadName.c_str(), this->thread);
		} 	
	}
 	
}

RNThread::~RNThread(){
	pthread_mutex_destroy(&mutexLocker);
	delete func;
}

void* RNThread::run(void* arg){
	RNThread *self = (RNThread*)arg;
	void* result = NULL;

	if(dynamic_cast<RNRetFunPointer<void*>*>(self->getFunctionPointer())){
		result = ((RNRetFunPointer<void*>*)self->getFunctionPointer())->invokeR();
	} else {
		self->getFunctionPointer()->invoke();
	}

	return result;
}

const char* RNThread::getThreadName() const{
	return this->threadName.c_str();
}

RNFunPointer* RNThread::getFunctionPointer() const{
	return this->func;
}

int RNThread::join(void** ret){
	int result;
	if((result = pthread_join(this->thread, ret)) != 0){
		switch (result){
			case ESRCH:
				RNUtils::printLn("RNThread::join: Error in join: No such thread found");
				result = NO_SUCH_THREAD;
				break;
			case EINVAL:
				RNUtils::printLn("RNThread::join: Error in join: Thread is detached or another thread is waiting");
				result = INVALID;
				break;
			case EDEADLK:
				RNUtils::printLn("RNThread::join: Error in join: Trying to join on self");
				result = JOIN_SELF;
				break;
		}
	}
	return result;
}

int RNThread::detach(void){
	int result;
	if((result = pthread_detach(this->thread)) != 0){
		switch (result){
			case ESRCH:
				RNUtils::printLn("RNThread::detach: Error in detach: No such thread found");
				result = NO_SUCH_THREAD;
				break;
			case EINVAL:
				RNUtils::printLn("RNThread::detach: Error in detach: Thread is already detached");
				result = ALREADY_DETACHED;
				break;
		}
	}
	this->joinable = false;
	return result;
}

void RNThread::cancel(void){
	pthread_cancel(this->thread);
}

void RNThread::stop(void){
	isRunning = false;
}

pid_t RNThread::getPID() const{
	return this->processId;
}

pthread_t RNThread::getThread() const{
	return this->thread;
}

int RNThread::lock(){
	return pthread_mutex_lock(&mutexLocker);
}

int RNThread::tryLock(){
	return pthread_mutex_trylock(&mutexLocker);
}

int RNThread::unlock(){
	return pthread_mutex_unlock(&mutexLocker);
}

void RNThread::threadStarted(){
	this->isStarted = true;
	processId = getpid();
	if(this->threadName.size() == 0){
		RNUtils::printLn("Anonymous thread (%ld) is running with processId %d", this->thread, this->processId);
	} else {
		RNUtils::printLn("Thread %s (%ld) is running with processId %d", this->threadName.c_str(), this->thread, this->processId);
	}
}

bool RNThread::isThreadStarted() const{
	return this->isStarted;
}

bool RNThread::getJoinable() const{
	return this->joinable;
}

bool RNThread::isThreadRunning() const{
	return this->isRunning;
}

void RNThread::threadFinished(){
	this->hasFinished = true;

	if(this->threadName.size() == 0){
		RNUtils::printLn("Anonymous thread (%ld) with processId %d has has finished", this->thread, this->processId);
	} else {
		RNUtils::printLn("Thread %s (%ld) with processId %d has finished", this->threadName.c_str(), this->thread, this->processId);
	}
}

bool RNThread::hasThreadFinished() const{
	return this->hasFinished;
}

void RNThread::setThreadName(const char* name){
	this->threadName = std::string(name);
}