#ifndef RN_LASER_TASK_H
#define RN_LASER_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"

class RNLaserTask : public RNRecurrentTask{
public:
	RNLaserTask(GeneralController* gn, const char* name = "Laser Task", const char* description = "");
	~RNLaserTask();
	virtual void task();
	virtual void onKilled();
	virtual void init();
private:
	
private:
	GeneralController* gn;
	LaserScan *laserDataScan;
	ArLaserConnector *laserConnector;
	ArLaser *laser;

	bool enableLocalization;
};

#endif