#ifndef RN_EKF_TASK_H
#define RN_EKF_TASK_H

#include "RNLocalizationTask.h"
#include "UDPServer.h"

class RNEkfTask : public RNLocalizationTask, public UDPServer{
public:
	RNEkfTask(const GeneralController* gn, const char* name = "Extended Kalman Localization Task", const char* description = "");
	~RNEkfTask();
	virtual void task();
	virtual void kill();
	virtual void init();
private:
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);  //temporal function

	void getObservations(Matrix& observations);
	void landmarkObservation(const Matrix& xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle);
	Matrix fixFilterGain(const Matrix wk);
private:
	static const double CAMERA_ERROR_POSITION_X;
	static const double CAMERA_ERROR_POSITION_Y;
	static const int STATE_VARIABLES;
	int laserLandmarksCount;
	int cameraLandmarksCount;

	double laserTMDistance;
	double laserTMAngle;
	double cameraTMDistance;
	double cameraTMAngle;

	bool enableLocalization;

	MapSector* currentSector;

	std::FILE* test;
	std::FILE* lmfile;
	Matrix xk;		// current position
	Matrix xk_1;	// previous position

	Matrix Ak;
	Matrix Bk;
	Matrix pk1;
	Matrix Hk;
	Matrix Pk;
};

#endif