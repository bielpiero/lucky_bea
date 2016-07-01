#include "GeneralController.h"
#include "RNRecurrentTaskMap.h"

RNRecurrentTaskMap::RNRecurrentTaskMap(GeneralController* gn){
	this->gn = gn;
	tasks = new std::vector<RNRecurrentTask*>();
}

RNRecurrentTaskMap::~RNRecurrentTaskMap(){

}

void RNRecurrentTaskMap::addTask(RNRecurrentTask* task){

}

void RNRecurrentTaskMap::removeTask(RNRecurrentTask* task){

}

void RNRecurrentTaskMap::removeAllTasks(){

}

RNRecurrentTask* RNRecurrentTaskMap::findTask(const char* taskName){

}

void RNRecurrentTaskMap::startTask(RNRecurrentTask* task){

}

void RNRecurrentTaskMap::startTask(const char* taskName){

}

void RNRecurrentTaskMap::startAllTasks(){

}

void RNRecurrentTaskMap::stopTask(RNRecurrentTask* task){

}

void RNRecurrentTaskMap::stopTask(const char* taskName){

}

void RNRecurrentTaskMap::stopAllTasks(){

}