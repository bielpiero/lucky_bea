#ifndef RN_LOCALIZATION_TASK_H
#define RN_LOCALIZATION_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include "Matrix.h"

#define STATE_VARIABLES 3

class RNLocalizationTask : public RNRecurrentTask{
public:
	RNLocalizationTask(const char* name = "Localization Task", const char* description = "");
	virtual void task();
	virtual void onKilled();
private:
	void init();
	void getObservations(Matrix& observations);
	void landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle);
private:
	float alpha;

	bool enableLocalization;

	Matrix Ak;
	Matrix Bk;
	Matrix pk1;
	Matrix Hk;
	Matrix Pk;
};

#endif