#ifndef RN_GLOBAL_LOCALIZATION_TASK_H
#define RN_GLOBAL_LOCALIZATION_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmarkList.h"
#include "GeneralController.h"

#define STATE_VARIABLES 3
#define LOCATION_HISTORY 10

class RNGlobalLocalizationTask : public RNRecurrentTask{
public:
	RNGlobalLocalizationTask(const char* name = "Global Localization Task", const char* description = "");
	~RNGlobalLocalizationTask();
protected:
	virtual void task();
	virtual void onKilled();

private:
	//std::vector<Trio<int, int, int> > *history;
	GeneralController* gn;
	bool enableGlobalLocalization;
};

#endif