#ifndef RN_TOUR_TASK_H
#define RN_TOUR_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"

class RNTourTask : public RNRecurrentTask{

public:
	RNTourTask(const GeneralController* gn, const char* name = "Tour Task", const char* description = "Doris tour task");
	~RNTourTask();
	virtual void task();
	virtual void onKilled();
private:
	GeneralController* gn;
};

#endif