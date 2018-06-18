#include "RNTimer.h"

RNTimer::RNTimer(const char* name, const char* description){
	this->name = "RNTimer: " + std::string(name);
    this->description = std::string(description);
    setThreadName(this->name.c_str());
    running = goRequested = killed = false;
    this->msecs = std::chrono::milliseconds(0);
    timeoutCB = NULL;
    create();
}

RNTimer::~RNTimer(){

}

void* RNTimer::runThread(void* object){
	threadStarted();
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(RNUtils::ok() and isRunning){
        testCancel(); 
        bool doit;
        std::chrono::milliseconds msecs;
        lock();
        doit = goRequested;
        msecs = this->msecs;
        unlock();
        if(doit){
            lock();
            running = true;
            unlock();
            std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
            if( std::chrono::duration_cast<std::chrono::milliseconds>(now - begin) >= msecs){
            	if(timeoutCB != NULL){
            		timeoutCB->invoke();
            	}
            	begin = now;
            } 
            lock();
            running = false;
            unlock();
        }
        RNUtils::sleep(1);
    }
    stop();
    threadFinished();
    return NULL;
}

void RNTimer::addTimeoutCB(RNFunPointer* timeoutCB){
	this->timeoutCB = timeoutCB;
}

void RNTimer::start(){
	lock();
    goRequested = true;
    running = true;
    killed = false;
    begin = std::chrono::high_resolution_clock::now();
    unlock();
}
void RNTimer::stop(){
	lock();
    goRequested = false;
    running = false;
    killed = true;
    unlock();
}
void RNTimer::setInterval(long long int msecs){
	lock();
	this->msecs =  std::chrono::milliseconds(msecs);
	unlock();
}

void RNTimer::reset(){
	stop();
	start();
}