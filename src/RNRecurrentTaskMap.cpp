#include "RNRecurrentTaskMap.h"
#include "GeneralController.h"

RNRecurrentTaskMap::RNRecurrentTaskMap(GeneralController* gn){
	this->gn = gn;
	tasks = new std::vector<RNRecurrentTask*>();
}

RNRecurrentTaskMap::~RNRecurrentTaskMap(){
	removeAllTasks();
	delete tasks;
}

void RNRecurrentTaskMap::addTask(RNRecurrentTask* task){
    task->setController(gn);
    tasks->push_back(task);
    RNUtils::printLn("Task %s added", task->getTaskName().c_str());
}

void RNRecurrentTaskMap::removeTask(RNRecurrentTask* task){
	if(task != NULL){
		int index = RN_NONE;
		for(int i = 0; i < tasks->size() and (index == RN_NONE); i++){
			if(tasks->at(i)->getTaskName() == task->getTaskName()){
				index = i;
			}
		}
		tasks->erase(tasks->begin() + index);
		task->kill();
		delete task;
	}
}

void RNRecurrentTaskMap::removeAllTasks(){
	for(int i = 0; i < tasks->size(); i++){
		tasks->at(i)->kill();
		delete tasks->at(i);
	}
	tasks->clear();
}

RNRecurrentTask* RNRecurrentTaskMap::findTask(const char* taskName){
	RNRecurrentTask* result = NULL;
	for(int i = 0; i < tasks->size(); i++){
		if(tasks->at(i)->getTaskName() == std::string(taskName)){
			result = tasks->at(i);
		}
	}
	return result;
}

void RNRecurrentTaskMap::startTask(RNRecurrentTask* task){
	if(task != NULL){
		task->go();
	}

}

void RNRecurrentTaskMap::startTask(const char* taskName){
	RNRecurrentTask* task = findTask(taskName);
	if(task != NULL){
		task->go();
	}
}

void RNRecurrentTaskMap::startAllTasks(){
	for(int i = 0; i < tasks->size(); i++){
		tasks->at(i)->go();
	}
	RNUtils::printLn("All Asynchornous tasks are started...");

}

void RNRecurrentTaskMap::stopTask(RNRecurrentTask* task){
	if(task != NULL){
		task->kill();
	}
}

void RNRecurrentTaskMap::stopTask(const char* taskName){
	RNRecurrentTask* task = findTask(taskName);
	if(task != NULL){
		task->kill();
	}
}

void RNRecurrentTaskMap::stopAllTasks(){
	for(int i = 0; i < tasks->size(); i++){
		tasks->at(i)->kill();
	}
}