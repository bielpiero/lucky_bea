#ifndef RN_PF_LOCALIZATION_TASK_H
#define RN_PF_LOCALIZATION_TASK_H

#include "RNLocalizationTask.h"
#include "UDPServer.h"

class RNPKalmanLocalizationTask : public RNLocalizationTask, public UDPServer{
public:
	RNPKalmanLocalizationTask(const GeneralController* gn, const char* name = "Possibilistic Kalman-Filter Localization Task", const char* description = "");
	~RNPKalmanLocalizationTask();
	virtual void task();
	virtual void kill();
	virtual void init();
private:
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);  //temporal function

	void getObservations(Matrix& observations_sup, Matrix& observations_inf);
	void landmarkObservation(const Matrix& xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle);
	Matrix fixFilterGain(const Matrix wk);
private:
	
	static const double CAMERA_ERROR_POSITION_X;
	static const double CAMERA_ERROR_POSITION_Y;
	static const float MULTIPLIER_FACTOR;

	static const float incertidumbre_odom_dist_sup;
	static const float incertidumbre_odom_dist_inf;
	static const float incertidumbre_odom_angl_sup;
	static const float incertidumbre_odom_angl_inf;

	static const float incertidumbre_camera_angl_sup;
	static const float incertidumbre_camera_angl_inf;

	static const float incertidumbre_laser_dist_sup;
	static const float incertidumbre_laser_dist_inf;
	static const float incertidumbre_laser_angl_sup;
	static const float incertidumbre_laser_angl_inf;

	int laserLandmarksCount;
	int cameraLandmarksCount;

	double laserTMDistance;
	double laserTMAngle;
	double cameraTMDistance;
	double cameraTMAngle;

	bool enableLocalization;


	std::FILE* test;
	std::FILE* test2;
	std::FILE* test3;

	// k = current
	Matrix xk_sup;		// current position
	Matrix xk_inf;
	Matrix xk_pred_sup;
	Matrix xk_pred_inf;
	Matrix x_sup_1;		// previous position
	Matrix x_inf_1;

	Matrix Ak;
	Matrix Bk;
	Matrix Hk;

	Matrix P_sup_1;
	Matrix P_inf_1;
	Matrix Pk_pred_sup;
	Matrix Pk_pred_inf;
	Matrix Pk_sup;
	Matrix Pk_inf;

	Matrix R_sup;
    Matrix R_inf;
	Matrix Q_sup;
	Matrix Q_inf;
	MapSector* currentSector;
	Matrix CG;

	vector<Matrix> v_single_inno_sup, v_single_inno_inf, v_single_H;
};

#endif
