#ifndef RN_RECURRENT_TASK_H
#define RN_RECURRENT_TASK_H

#include "RNAsyncTask.h"

class GeneralController;

class RNRecurrentTask : public RNAsyncTask{
public:
	RNRecurrentTask(const char* name = "", const char* description = "");
	~RNRecurrentTask();
	virtual void task() = 0;

	void setController(GeneralController* gn);
private:
	GeneralController* gn;
	std::string name;
	std::string description;
	bool running;
	bool goRequested;
	bool killed;
};

#endif