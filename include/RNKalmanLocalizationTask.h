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
private:
	void init();
	void getObservations(Matrix& observations);
	void landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle);
private:
	
	int laserLandmarksCount;
	int cameraLandmarksCount;
	int rfidLandmarksCount;

	float alpha;

	bool enableLocalization;

	Matrix Ak;
	Matrix Bk;
	Matrix pk1;
	Matrix Hk;
	Matrix Pk;
};

#endif