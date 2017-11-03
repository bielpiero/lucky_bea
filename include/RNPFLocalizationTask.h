#ifndef RN_PF_LOCALIZATION_TASK_H
#define RN_PF_LOCALIZATION_TASK_H

#include "RNLocalizationTask.h"

class RNPFLocalizationTask : public RNLocalizationTask{
public:
	RNPFLocalizationTask(const GeneralController* gn, const char* name = "Particle-Filter Localization Task", const char* description = "");
	~RNPFLocalizationTask();
	virtual void task();
	virtual void onKilled();
	virtual void init();
private:
	
private:
	
	int laserLandmarksCount;
	int cameraLandmarksCount;
	int rfidLandmarksCount;

	float alpha;

	bool enableLocalization;
};

#endif