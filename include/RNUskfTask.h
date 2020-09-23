#ifndef RN_USKF_TASK_H
#define RN_USKF_TASK_H

#include "RNLocalizationTask.h"
#include "UDPServer.h"


class RNUskfTask : public RNLocalizationTask, public UDPServer{
public:
	RNUskfTask(const GeneralController* gn, const char* name = "Unscented Schmidt-Kalman Localization Task", const char* description = "");
	~RNUskfTask();
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
	static const double KAPPA;
	static const double LAMBDA;

	static const int SV;
	static const int BV;
	static const int SV_AUG;
	static const int SV_AUG_SIGMA;
	static const int BIAS_VARIABLES;

	
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
	Matrix xkAug_1;
	Matrix pk1;
	Matrix Pk;
	Matrix PkAug;
	Matrix XSigPred;
	Matrix XSigPredAug;
	double* wm;
	double* wc;

	Matrix b;
	Matrix B;
	Matrix Pxbk;
	Matrix pxbk1;
};

#endif