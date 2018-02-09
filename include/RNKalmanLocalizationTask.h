#ifndef RN_KALMAN_LOCALIZATION_TASK_H
#define RN_KALMAN_LOCALIZATION_TASK_H

#include "RNLocalizationTask.h"
//#include "UDPServer.h"

#define STATE_VARIABLES 3

class RNKalmanLocalizationTask : public RNLocalizationTask{
public:
	RNKalmanLocalizationTask(const GeneralController* gn, const char* name = "Kalman Localization Task", const char* description = "");
	~RNKalmanLocalizationTask();
	virtual void task();
	virtual void onKilled();
	virtual void init();
private:
	
	void getObservations(Matrix& observations);
	void landmarkObservation(const Matrix& xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle);
private:
	static const double MAX_LASER_DISTANCE_ERROR;
	static const double MAX_LASER_ANGLE_ERROR;
	static const double MAX_CAMERA_DISTANCE_ERROR;
	static const double MAX_CAMERA_ANGLE_ERROR;
	static const double CAMERA_ERROR_POSITION_X;
	static const double CAMERA_ERROR_POSITION_Y;
	int laserLandmarksCount;
	int cameraLandmarksCount;
	int rfidLandmarksCount;

	double laserTMDistance;
	double laserTMAngle;
	double cameraTMDistance;
	double cameraTMAngle;
	double rfidTMDistance;
	double rfidTMAngle;

	bool enableLocalization;
	//UDPServer* receiver;

	std::FILE* test;
	Matrix xk;		// current position
	Matrix xk_1;	// previous position

	Matrix Ak;
	Matrix Bk;
	Matrix pk1;
	Matrix Hk;
	Matrix Pk;
};

#endif