#ifndef RN_LASER_TASK_H
#define RN_LASER_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"

#define MIN_INDEX_LASER_SECURITY_DISTANCE 90
#define MAX_INDEX_LASER_SECURITY_DISTANCE 120

class RNLaserTask : public RNRecurrentTask{
public:
	RNLaserTask(GeneralController* gn, const char* name = "Laser Task", const char* description = "");
	~RNLaserTask();
	virtual void task();
private:
	void getLaserScan(void);
	void securityDistanceChecker();
	void getReflectiveLandmarks();

private:
	GeneralController* gn;
	LaserScan *laserDataScan;
	ArLaserConnector *laserConnector;
	ArLaser *laser;

	RNLandmarkList* laserLandmarks;

	bool laserActive;

	static const double SECURITY_DISTANCE;
	static const double LASER_MAX_RANGE;
	static const double LANDMARK_RADIUS;

	/** VICTOR */
	std::FILE* test2;
};

#endif
