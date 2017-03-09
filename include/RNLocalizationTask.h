#ifndef RN_LOCALIZATION_TASK_H
#define RN_LOCALIZATION_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include "Matrix.h"

#define STATE_VARIABLES 3

class RNLocalizationTask : public RNRecurrentTask{
public:
	RNLocalizationTask(const char* name = "Localization Task", const char* description = "") : RNRecurrentTask(name, description){

	}
	~RNLocalizationTask(){

	}
protected:
	virtual void task(){

	}
	
	virtual void onKilled(){

	}
};

#endif