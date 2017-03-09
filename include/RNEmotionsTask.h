#ifndef RN_EMOTIONS_TASK_H
#define RN_EMOTIONS_TASK_H

#include "RNRecurrentTask.h"

class RNEmotionsTask : public RNRecurrentTask{
public:
	RNEmotionsTask(const char* name = "Emotions Task", const char* description = "Doris Feelings");
	~RNEmotionsTask();
	virtual void task();
	virtual void onKilled();
};

#endif