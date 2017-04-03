#ifndef RN_DIALOGS_TASK_H
#define RN_DIALOGS_TASK_H

#include "RNRecurrentTask.h"

class RNDialogsTask : public RNRecurrentTask{
public:
	RNDialogsTask(const char* name = "Dialogs Task", const char* description = "Doris Speech");
	~RNDialogsTask();
	virtual void task();
	virtual void onKilled();
};

#endif