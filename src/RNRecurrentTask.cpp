#include "RNRecurrentTask.h"

RNRecurrentTask::RNRecurrentTask(const char* name, const char* description){
    this->name = "RNRecurrentTask: " + std::string(name);
    this->description = std::string(description);
    setThreadName(this->name.c_str());
    running = goRequested = killed = false;
    create();
}

RNRecurrentTask::~RNRecurrentTask(){
    kill();
}

void RNRecurrentTask::setController(GeneralController* gn){
	this->gn = gn;
}

void RNRecurrentTask::go(){
    lock();
    goRequested = true;
    running = true;
    killed = false;
    unlock();
}

int RNRecurrentTask::done(){
    int result = 1;
    if(running){
        result = 0;
    } else if(killed){
        result = 2;
    }
    return result;
}

void RNRecurrentTask::reset(){
    if(running){
        kill();
        create();
        go();
    }
}

void RNRecurrentTask::kill(){
    lock();
    goRequested = false;
    running = false;
    killed = true;
    unlock();
    cancel();
}

void* RNRecurrentTask::runThread(void* object){
    threadStarted();
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(RNUtils::ok() and isRunning){
        while(not goRequested){
            RNUtils::sleep(10);
        }
        lock();
        running = true;
        unlock();
        task();
    }
    threadFinished();
    return NULL;
}

std::string RNRecurrentTask::getTaskName(){
    return this->name;
}