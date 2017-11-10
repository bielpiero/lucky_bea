#ifndef RN_LASER_TASK_H
#define RN_LASER_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"

#define MIN_INDEX_LASER_SECURITY_DISTANCE 90
#define MAX_INDEX_LASER_SECURITY_DISTANCE 120

#define DEFAULT_SECURITY_DISTANCE_WARNING_TIME 30
#define DEFAULT_SECURITY_DISTANCE_STOP_TIME 60

class RNLaserTask : public RNRecurrentTask{
public:
	RNLaserTask(GeneralController* gn, const char* name = "Laser Task", const char* description = "");
	~RNLaserTask();
	virtual void task();
	virtual void onKilled();
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

	static const float SECURITY_DISTANCE;
	static const float LASER_MAX_RANGE;
	static const float LANDMARK_RADIUS;
};

#endif