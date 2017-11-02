#ifndef RN_KALMAN_LOCALIZATION_TASK_H
#define RN_KALMAN_LOCALIZATION_TASK_H

#include "RNLocalizationTask.h"

#define STATE_VARIABLES 3

class RNKalmanLocalizationTask : public RNLocalizationTask{
public:
	RNKalmanLocalizationTask(const char* name = "Kalman Localization Task", const char* description = "");
	~RNKalmanLocalizationTask();
	virtual void task();
	virtual void onKilled();
	virtual void init();
private:
	
	void getObservations(Matrix& observations);
	void landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle);
private:
	static const float MAX_LASER_DISTANCE_ERROR;
	static const float MAX_LASER_ANGLE_ERROR;
	static const float MAX_CAMERA_DISTANCE_ERROR;
	static const float MAX_CAMERA_ANGLE_ERROR;
	static const float CAMERA_ERROR_POSITION_X;
	static const float CAMERA_ERROR_POSITION_Y;
	int laserLandmarksCount;
	int cameraLandmarksCount;
	int rfidLandmarksCount;

	float laserTMDistance;
	float laserTMAngle;
	float cameraTMDistance;
	float cameraTMAngle;
	float rfidTMDistance;
	float rfidTMAngle;

	bool enableLocalization;

	std::FILE* test;

	Matrix Ak;
	Matrix Bk;
	Matrix pk1;
	Matrix Hk;
	Matrix Pk;
};

#endif