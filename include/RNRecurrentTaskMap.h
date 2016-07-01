#ifndef RN_RECURRENT_TASK_MAP_H
#define RN_RECURRENT_TASK_MAP_H

#include "RNRecurrentTask.h"
#include <vector>

class GeneralController;

class RNRecurrentTaskMap{
private:
	std::vector<RNRecurrentTask*>* tasks;
public:
	RNRecurrentTaskMap(GeneralController* gn);
	virtual ~RNRecurrentTaskMap();

	set

	addTask(RNRecurrentTask* task);
	removeTask(RNRecurrentTask* task);
	findTask(const char* taskName);

	startTask(RNRecurrentTask* task);
	startTask(const char* taskName);
	startAllTasks();

	stopTask(RNRecurrentTask* task);
	stopTask(const char* taskName);
	stopAllTasks();
};

#endif