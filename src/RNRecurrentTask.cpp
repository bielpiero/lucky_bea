#include "RNRecurrentTask.h"

RNRecurrentTask::RNRecurrentTask(const RobotNode* rn, const char* name, const char* description){
    this->name = "RNRecurrentTask: " + std::string(name);
    this->description = std::string(description);
    setThreadName(this->name.c_str());
    executingTask = goRequested = killed = false;
    this->rn = (RobotNode*)rn;
    create();
}

RNRecurrentTask::~RNRecurrentTask(){
    kill();
}

void RNRecurrentTask::setController(RobotNode* rn){
	this->rn = rn;
}

void RNRecurrentTask::go(){
    lock();
    goRequested = true;
    executingTask = true;
    killed = false;
    unlock();
}

int RNRecurrentTask::done(){
    int result = 1;
    if(executingTask){
        result = 0;
    } else if(killed){
        result = 2;
    }
    return result;
}

void RNRecurrentTask::reset(){
    if(not isThreadRunning()){
        create();
        go();
    } else {
        kill();
        go();
    }
}

void RNRecurrentTask::kill(){
    if(isThreadRunning()){
        RNUtils::printLn("Thread %s will be killed...", this->getTaskName().c_str());
        lock();
        goRequested = false;
        executingTask = false;
        killed = true;
        unlock();       
        RNUtils::printLn("Thread %s has been killed...", this->getTaskName().c_str());    
        //onKilled();
    }
}

void RNRecurrentTask::waitUtilTaskFinished(){
    while(not hasThreadFinished()){
        RNUtils::sleep(5);
    }
}

void* RNRecurrentTask::runThread(void* object){
    threadStarted();
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(RNUtils::ok() and isRunning){
        testCancel(); 
        bool doit;
        lock();
        doit = goRequested;
        unlock();
        if(doit){
            lock();
            executingTask = true;
            unlock();
            task();
            lock();
            executingTask = false;
            unlock();
        }
        RNUtils::sleep(30);
    }
    stop();
    threadFinished();
    return NULL;
}

std::string RNRecurrentTask::getTaskName(){
    return this->name;
}