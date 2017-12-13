#ifndef RN_FACTORY_SENSORS_TASK_H
#define RN_FACTORY_SENSORS_TASK_H

#include "RNRecurrentTask.h"

class RNFactorySensorsTask : public RNRecurrentTask{
public:
	RNFactorySensorsTask(const RobotNode* gn, const char* name = "Factory Sensors Task", const char* description = "");
	~RNFactorySensorsTask();
protected:
	virtual void task();
	virtual void onKilled();

private:
	void getRawRobotPosition(void);
	void getBumpersStatus(void);
	void computePositionFromEncoders(void);
	void getRawPoseFromOdometry(ArPose* rawPose);
	void getSonarsScan(void);

private:
	bool isFirstFakeEstimation;

	ArPose prevRawPose;

	double prevDistance;
    double prevRads;
    double prevVel;
    double prevRotVel;

    double deltaDistance;
    double deltaDegrees;
};

#endif