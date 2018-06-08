#ifndef RN_KALMAN_LOCALIZATION_TASK_H
#define RN_KALMAN_LOCALIZATION_TASK_H

#include "RNLocalizationTask.h"
#include "UDPServer.h"

#define STATE_VARIABLES 3

class RNKalmanLocalizationTask : public RNLocalizationTask, public UDPServer{
public:
	RNKalmanLocalizationTask(const GeneralController* gn, const char* name = "Kalman Localization Task", const char* description = "");
	~RNKalmanLocalizationTask();
	virtual void task();
	virtual void onKilled();
	virtual void init();
private:
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);  //temporal function

	void getObservations(Matrix& observations);
	void landmarkObservation(const Matrix& xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle);
	Matrix fixFilterGain(const Matrix wk, const Matrix z);
private:
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