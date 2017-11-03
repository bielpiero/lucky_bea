#ifndef RN_LOCALIZATION_TASK_H
#define RN_LOCALIZATION_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmarkList.h"
#include "Matrix.h"
#include "GeneralController.h"

#define STATE_VARIABLES 3

class RNLocalizationTask : public RNRecurrentTask{
public:
	RNLocalizationTask(const GeneralController* gn, const char* name = "Localization Task", const char* description = "") : RNRecurrentTask(gn, name, description){
		this->gn = (GeneralController*)gn;
	}
	~RNLocalizationTask(){

	}
protected:
	GeneralController* gn;
protected:
	virtual void task() = 0;
	virtual void onKilled() = 0;

public:
	virtual void init() = 0;
};

#endif