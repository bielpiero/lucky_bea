#include "RNUkfTask.h"

const double RNUkfTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNUkfTask::CAMERA_ERROR_POSITION_Y = -0.014;
const double RNUkfTask::ALPHA = 1.0e-3;
const double RNUkfTask::BETA = 2;
const double RNUkfTask::KAPPA = 0;
const double RNUkfTask::LAMBDA = ALPHA*ALPHA*((double)SV_AUG + KAPPA) - (double)SV_AUG;

RNUkfTask::RNUkfTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	currentSector = NULL;
	test = std::fopen("laser_camera.txt","w+");
	this->startThread();
	wm = NULL;
	wc = NULL;
}

RNUkfTask::~RNUkfTask(){
	if(test != NULL){
		std::fclose(test);
	}
}

void RNUkfTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){

		xkAug = Matrix(SV_AUG, 1);
		PkAug = Matrix(SV_AUG, SV_AUG);
		XSigPred = Matrix(SV, SV_AUG_SIGMA);
		XSigPredAug = Matrix(SV_AUG, SV_AUG_SIGMA);

		if(wm != NULL){
			delete[] wm;
		}
		if(wc != NULL){
			delete[] wc;
		}
		wm = new double[SV_AUG_SIGMA];
		wc = new double[SV_AUG_SIGMA];

		wm[0] = (LAMBDA / ((double)SV_AUG + LAMBDA));
		wc[0] = (LAMBDA / ((double)SV_AUG + LAMBDA)) + (1 - ALPHA*ALPHA + BETA);
		for(int i = 1; i < SV_AUG_SIGMA; i++){
			wm[i] = 1/(2*((double)SV_AUG + LAMBDA));
			wc[i] = 1/(2*((double)SV_AUG + LAMBDA));
		}

		Pk = gn->getP();
		/*if(currentSector != NULL){
			delete currentSector;
		}*/
		currentSector = gn->getCurrentSector();
		laserLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		cameraLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);


		laserTMDistance = gn->getLaserDistanceAlpha() / std::sqrt(gn->getLaserDistanceVariance());
		laserTMAngle = gn->getLaserAngleAlpha() / std::sqrt(gn->getLaserAngleVariance());
		//cameraTMDistance = gn->getCameraDistanceAlpha() / std::sqrt(gn->getCameraDistanceVariance());
		cameraTMDistance = 0.1;
		cameraTMAngle = gn->getCameraAngleAlpha() / std::sqrt(gn->getCameraAngleVariance());


		xk = gn->getRawEncoderPosition();
		xk.print();
		
		enableLocalization = true;

	} else{
		enableLocalization = false;
	}
}

void RNUkfTask::kill(){
	enableLocalization = false;
	RNRecurrentTask::kill();
}

void RNUkfTask::prediction(){

	double deltaDistance = 0.0, deltaAngle = 0.0;
	gn->getIncrementPosition(&deltaDistance, &deltaAngle);

	xkAug(0, 0) = xk(0, 0);
	xkAug(1, 0) = xk(1, 0);
	xkAug(2, 0) = xk(2, 0);
	xkAug(3, 0) = 0;
	xkAug(4, 0) = 0;

	for(int i = 0; i < SV; i++){
		for(int j= 0; j < SV; j++){
			PkAug(i, j) = Pk(i, j);
		}
	}
	/*if(deltaDistance == 0.0 and deltaAngle == 0.0){
		PkAug(3, 3) = 0.0;
		PkAug(4, 4) = 0.0;
	} else {*/
		PkAug(3, 3) = gn->getQ()(0, 0);
		PkAug(4, 4) = gn->getQ()(1, 1);
	//}

	Matrix L = PkAug.chol();
	
	XSigPredAug.setCol(0, xkAug);

	// sigma points
	double c = std::sqrt((double)SV_AUG + LAMBDA);
	for(int i = 1; i <= SV_AUG; i++){
		XSigPredAug.setCol(i, xkAug + c * L.col(i - 1));
		XSigPredAug(2, i) = RNUtils::fixAngleRad(XSigPredAug(2, i));

		XSigPredAug.setCol(i + SV_AUG, xkAug - c * L.col(i - 1));
		XSigPredAug(2, i + SV_AUG) = RNUtils::fixAngleRad(XSigPredAug(2, i + SV_AUG));
	}

	//Prediction of every sigma point
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		RNUtils::getOdometryPose(XSigPredAug.col(i), deltaDistance, deltaAngle, xk_1);
		XSigPred.setCol(i, xk_1);
	}
	// Mean of state
	Matrix xk_mean(3, 1);
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		xk_mean = xk_mean + wm[i] * XSigPred.col(i);
		xk_mean(2, 0) = RNUtils::fixAngleRad(xk_mean(2, 0));
	}
	xk_1 = xk_mean;
	//printf("X(k + 1|k) mean\n");
	//xk_1.print();

	//printf("XSigPred\n");
	//XSigPred.print();

	//covariance matrix
	Matrix Pyy(SV, SV);
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		Matrix diff = XSigPred.col(i) - xk_1;
		diff(2, 0) = RNUtils::fixAngleRad(diff(2, 0));
		Pyy = Pyy + wc[i] * (diff * ~diff);
	}
	Pk = Pyy;
}

void RNUkfTask::obtainMeasurements(Matrix& zkli, std::vector<int>& ids){
	bool idsProcessed = false;
	int totalLandmarks = 0;
	int laserIndex = 0, cameraIndex = 0;
	if(gn->isLaserSensorActivated()){
		totalLandmarks += laserLandmarksCount;
		cameraIndex += laserLandmarksCount;	
	}

	if(gn->isCameraSensorActivated()){
		totalLandmarks += cameraLandmarksCount;

	}

	Matrix results(2*laserLandmarksCount + cameraLandmarksCount, SV_AUG_SIGMA); 
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		Matrix zkl(totalLandmarks, 4);
		predictMeasurements(zkl, XSigPred.col(i));
		for(int j = 0; j < laserLandmarksCount; j++){
			results(2*j, i) = zkl(j, 0);
			results(2*j + 1, i) = zkl(j, 1);
			if(not idsProcessed){
				ids.push_back((int)zkl(j, 3));
			}
			
		}

		for(int j = cameraIndex; j < totalLandmarks; j++){
			results(j, i) = zkl(j, 1);
			if(not idsProcessed){
				ids.push_back((int)zkl(j, 3));
			}
		}

		idsProcessed = true;
	}

	zkli = results;
	
}


void RNUkfTask::task(){
	if(enableLocalization){
		int rsize = 0, vsize = 0;
		int activeRL = 0, activeVL = 0;
		pk1 = Pk;
		xk_1 = xk;

		//printf("\n\n\nNew Iteration:\n");
		gn->getRobotPosition();
		prediction();

		//printf("P(k + 1|k)\n");
		//Pk.print();
		
		if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
			printf("Error gordo...\n");
		}

		Matrix zkli;
		std::vector<int> ids;
		obtainMeasurements(zkli, ids);


		gn->lockLaserLandmarks();
		gn->lockVisualLandmarks();

		int totalLandmarks = 0;
		if(gn->isLaserSensorActivated()){
			activeRL = gn->getLaserLandmarks()->size();
			totalLandmarks += activeRL;
		}

		if(gn->isCameraSensorActivated()){
			activeVL = gn->getVisualLandmarks()->size();
			totalLandmarks += activeVL;
		}
		int sizeH = (gn->isLaserSensorActivated() ? 2 * activeRL : 0) + (gn->isCameraSensorActivated() ? activeVL : 0);
		Matrix currentR(sizeH, sizeH);
		Matrix zik_1(sizeH, 1);
		Matrix zi(sizeH, SV_AUG_SIGMA);
		Matrix zi_mean(sizeH, 1);
		if(sizeH > 1){
			int laserIndex = 0, cameraIndex = 0;
			if(gn->isLaserSensorActivated()){
				cameraIndex = 2 * activeRL;
				rsize = gn->getLaserLandmarks()->size();
				for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
					RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);
					zik_1(2 * i, 0) = lndmrk->getPointsXMean();
					zik_1(2 * i + 1, 0) = lndmrk->getPointsYMean();

					Matrix measure(2, 1);
					measure(0, 0) = lndmrk->getPointsXMean();
					measure(1, 0) = lndmrk->getPointsYMean();

					Matrix smallR(2, 2);
					smallR(0, 0) = gn->getLaserDistanceVariance();
					smallR(1, 1) = gn->getLaserAngleVariance();
					currentR(2 * i, 2 * i) = smallR(0, 0);
					currentR(2 * i + 1, 2 * i + 1) = smallR(1, 1);
					for(int j = 0; j < SV_AUG_SIGMA; j++){
						std::map<int, double> mahalanobisDistances = computeMahalanobis(j, measure, zkli, XSigPred.col(j), smallR, ids);

						auto minorDistance = std::min_element(mahalanobisDistances.begin(), mahalanobisDistances.end(), [](std::pair<int, double> x, std::pair<int, double> y){ return x.second < y.second; });
						if(minorDistance != mahalanobisDistances.end()){
							zi(2 * i, j) = zkli(2 * minorDistance->first, j);
							zi(2 * i + 1, j) = zkli(2 * minorDistance->first + 1, j);

							zi(2 * i + 1, j) = RNUtils::fixAngleRad(zi(2 * i + 1, j));
							
						}
					}
				}
			}

			if(gn->isCameraSensorActivated()){
				vsize = gn->getVisualLandmarks()->size();
				for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
					RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
					zik_1(i, 0) = lndmrk->getPointsYMean();

					currentR(cameraIndex + i, cameraIndex + i) = gn->getCameraAngleVariance();
					for(int j = 0; j < SV_AUG_SIGMA; j++){
						
						int zklIndex = RN_NONE;
						for(int k = 0; k < cameraLandmarksCount and (zklIndex == RN_NONE); k++){
							s_landmark* currLandmark = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_CAMERA_STR, ids[k + laserLandmarksCount]);
							if(currLandmark != NULL and currLandmark->type == XML_SENSOR_TYPE_CAMERA_STR){
								if(lndmrk != NULL and currLandmark->id == lndmrk->getMarkerId()){
									
									zklIndex = k + 2*laserLandmarksCount;
								}
							}
						}

						if(lndmrk != NULL and (lndmrk->getMapId() == currentSector->getMapId()) and (lndmrk->getSectorId() == currentSector->getId()) and zklIndex != RN_NONE){				
							zi(cameraIndex + i, 0) = zkli(zklIndex, j);
							//zl(cameraIndex + i, 0) = RNUtils::fixAngleRad(zl(cameraIndex + i, 0));
							//RNUtils::printLn("MapId: %d, SectorId: %d, markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {a: %f}", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(j + laserOffset, 0));
						}
					}

				}
			}

			for(int i = 0; i < SV_AUG_SIGMA; i++){
				zi_mean = zi_mean + wm[i] * zi.col(i);
			}

			//printf("zi_mean mean\n");
			//zi_mean.print();

			Matrix Sk(sizeH, sizeH);
			for(int i = 0; i < SV_AUG_SIGMA; i++){
				Matrix diff = zi.col(i) - zi_mean;
				Sk = Sk + wc[i]*(diff * ~diff);
			}
			Sk = Sk + currentR;

			Matrix Pxz(SV, sizeH);
			for(int i = 0; i < SV_AUG_SIGMA; i++){
				Matrix diff_x = XSigPred.col(i) - xk_1;
				diff_x(2, 0) = RNUtils::fixAngleRad(diff_x(2, 0));

				Matrix diff_z = zi.col(i) - zi_mean;
				Pxz = Pxz + wc[i] * (diff_x * ~diff_z);
			}

			Matrix Wk = Pxz * !Sk;

			//printf("Z(k + 1)\n");
			//zik_1.print();

			Matrix nu = zik_1 - zi_mean;
			for (int i = 0; i < activeRL; i++){
				double mdDistance = std::abs(nu(2 * i, 0) / std::sqrt(gn->getLaserDistanceVariance()));
				double mdAngle = std::abs(nu(2 * i + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));

				if (mdDistance > laserTMDistance or mdAngle > laserTMAngle){

					//RNUtils::printLn("landmark %d rejected...", indexFound);
					nu(2 * i, 0) = 0.0;
					nu(2 * i + 1, 0) = 0.0;
					rsize--;
				}
			}

			for(int i = 0; i < activeVL; i++){
				double mdAngle = std::abs(nu(cameraIndex + i, 0) / std::sqrt(gn->getCameraAngleVariance()));

				if (mdAngle > cameraTMAngle){
					//RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
					nu(cameraIndex + i, 0) = 0;
					vsize--;
				}
			}
			//printf("nu(k + 1)\n");
			//nu.print();

			xk = xk_1 + Wk * nu;
			xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));

			//printf("X(k + 1|k + 1) postfix\n");
			//xk.print();

			char bufferpk1[256], bufferpk[256];
			sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			Pk = Pk - Wk * Sk * ~Wk;
			//printf("P(k + 1|k + 1)\n");
			//Pk.print();
			if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
				printf("Error gordo...\n");
			}
			sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));

			gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
			char buffer[1024];
			sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize);
			if(test != NULL){
				fprintf(test, "%s", buffer);
			}
		} else {
			char bufferpk1[256], bufferpk[256];
			sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			//Pk = Pk;
			sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			xk = xk_1;
			//xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));
			gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
			char buffer[1024];
			sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize);
			if(test != NULL){
				fprintf(test, "%s", buffer);
			}
		}

		
		gn->getVisualLandmarks()->clear();
		gn->unlockVisualLandmarks();
		gn->unlockLaserLandmarks();
		

	} else {
		init();
	}
	RNUtils::sleep(20);
}

std::map<int, double> RNUkfTask::computeMahalanobis(const int& sigmaPoint, Matrix measure, const Matrix& zkli, const Matrix& xk, const Matrix& sr, const std::vector<int>& ids){
	std::map<int, double> mahalanobisDistances;

	for (int i = 0; i < laserLandmarksCount; i++){
		s_landmark* currLandmark = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_LASER_STR, ids[i]);
		Matrix smallHk(2, 3);
		if(currLandmark != NULL and currentSector->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){
			double landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, xk(0, 0), xk(1, 0));
			smallHk(0, 0) = -(currLandmark->xpos - xk(0, 0))/landmarkDistance;
			smallHk(0, 1) = -(currLandmark->ypos - xk(1, 0))/landmarkDistance;
			smallHk(0, 2) = 0.0;

			smallHk(1, 0) = (currLandmark->ypos - xk(1, 0))/std::pow(landmarkDistance, 2);
			smallHk(1, 1) = -(currLandmark->xpos - xk(0, 0))/std::pow(landmarkDistance, 2);
			smallHk(1, 2) = -1.0;
		}

		Matrix omega = smallHk * Pk * ~smallHk + sr;

		Matrix obs(2, 1);
		obs(0, 0) = zkli(2 * i, sigmaPoint);
		obs(1, 0) = zkli(2 * i + 1, sigmaPoint);

		Matrix mdk = ~(measure - obs) * omega * (measure - obs);

		mahalanobisDistances.emplace(i, std::sqrt(std::abs(mdk(0, 0))));
	}
	return mahalanobisDistances;
}


void RNUkfTask::predictMeasurements(Matrix& predictions, const Matrix& xk){
	int totalLandmarks = 0;
	int laserIndex = 0, cameraIndex = 0;
	if(gn->isLaserSensorActivated()){
		totalLandmarks += laserLandmarksCount;
		cameraIndex += laserLandmarksCount;	
	}

	if(gn->isCameraSensorActivated()){
		totalLandmarks += cameraLandmarksCount;
	}

	for(int k = 0; k < currentSector->landmarksSize(); k++){
		int zIndex = RN_NONE;
		double distance = 0, angle = 0;
		Matrix disp = Matrix(2, 1);
		s_landmark* landmark = currentSector->landmarkAt(k);
		if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
			double nrx, nry;
			RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk(2, 0), &nrx, &nry);
			disp(0, 0) = nrx;
			disp(1, 0) = nry;
		}
		landmarkObservation(xk, disp, landmark, distance, angle);

		if(gn->isLaserSensorActivated()){
			if(landmark->type == XML_SENSOR_TYPE_LASER_STR){
				zIndex = laserIndex;
				laserIndex++;
			}
		}
		if(gn->isCameraSensorActivated()){
			if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
				zIndex = cameraIndex;
				cameraIndex++;
			}
		}
		
		if(zIndex > RN_NONE){
			predictions(zIndex, 0) = distance;
			if(angle > M_PI){
				angle = angle - 2 * M_PI;
			} else if(angle < -M_PI){
				angle = angle + 2 * M_PI;
			}

			predictions(zIndex, 1) = angle;
			predictions(zIndex, 2) = landmark->zpos;
			predictions(zIndex, 3) = (double)landmark->id;
		}
		
	}
}

void RNUkfTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle){
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));
	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);
}

void RNUkfTask::OnMessageReceivedWithData(unsigned char* cad, int length){
	gn->lockVisualLandmarks();
	RNLandmarkList* markers = gn->getVisualLandmarks();
	markers->initializeFromString((char*)cad);
	gn->unlockVisualLandmarks();
}
