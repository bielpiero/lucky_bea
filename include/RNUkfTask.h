#ifndef RN_UKF_TASK_H
#define RN_UKF_TASK_H

#include "RNLocalizationTask.h"
#include "UDPServer.h"

#define SV 3
#define SV_AUG 5
#define SV_AUG_SIGMA 11


class RNUkfTask : public RNLocalizationTask, public UDPServer{
public:
	RNUkfTask(const GeneralController* gn, const char* name = "Unscented Kalman Localization Task", const char* description = "");
	~RNUkfTask();
	virtual void task();
	virtual void kill();
	virtual void init();
private:
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);  //temporal function
	void prediction();
	void obtainMeasurements(Matrix& zi, std::vector<int>& ids);
	void predictMeasurements(Matrix& predictions, const Matrix& xk);
	void landmarkObservation(const Matrix& xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle);
	std::map<int, double> computeMahalanobis(const int& sigmaPoint, Matrix measure, const Matrix& zkli, const Matrix& xk, const Matrix& sr, const std::vector<int>& ids);
	Matrix fixFilterGain(const Matrix wk);
private:
	static const double CAMERA_ERROR_POSITION_X;
	static const double CAMERA_ERROR_POSITION_Y;
	static const double ALPHA;
	static const double BETA;
	static const double LAMBDA;
	
	int laserLandmarksCount;
	int cameraLandmarksCount;

	double laserTMDistance;
	double laserTMAngle;
	double cameraTMDistance;
	double cameraTMAngle;

	bool enableLocalization;

	MapSector* currentSector;

	std::FILE* test;
	Matrix xk;		// current position
	Matrix xk_1;	// previous position

	Matrix xkAug;		// current position
	Matrix pk1;
	Matrix Pk;
	Matrix PkAug;
	Matrix XSigPred;
	Matrix XSigPredAug;
	std::vector<double> weights_m;
	std::vector<double> weights_c;
};

#endif