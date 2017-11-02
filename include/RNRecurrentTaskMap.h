#ifndef RN_RECURRENT_TASK_MAP_H
#define RN_RECURRENT_TASK_MAP_H

#include <vector>

#include "RNRecurrentTask.h"

class RNRecurrentTaskMap{
private:
	std::vector<RNRecurrentTask*>* tasks;
	RobotNode* rn;
public:
	RNRecurrentTaskMap(RobotNode* rn);
	virtual ~RNRecurrentTaskMap();

	void addTask(RNRecurrentTask* task);
	void removeTask(RNRecurrentTask* task);
	void removeAllTasks();
	RNRecurrentTask* findTask(const char* taskName);

	void startTask(RNRecurrentTask* task);
	void startTask(const char* taskName);
	void startAllTasks();

	void stopTask(RNRecurrentTask* task);
	void stopTask(const char* taskName);
	void stopAllTasks();
};

#endif