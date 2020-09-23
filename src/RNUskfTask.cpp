#include "RNUskfTask.h"

const int RNUskfTask::SV = 3;
const int RNUskfTask::BV = 5;
const int RNUskfTask::SV_AUG = SV + BV;
const int RNUskfTask::SV_AUG_SIGMA = 2*SV_AUG + 1;

const double RNUskfTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNUskfTask::CAMERA_ERROR_POSITION_Y = -0.014;
const double RNUskfTask::ALPHA = 1.0e-3;
const double RNUskfTask::BETA = 2;
const double RNUskfTask::KAPPA = 0;
const double RNUskfTask::LAMBDA = ALPHA*ALPHA*((double)SV_AUG + KAPPA) - (double)SV_AUG;



RNUskfTask::RNUskfTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	currentSector = NULL;
	test = std::fopen("laser_camera_uskf.txt","w+");
	this->startThread();
	wm = NULL;
	wc = NULL;
}

RNUskfTask::~RNUskfTask(){
	if(test != NULL){
		std::fclose(test);
	}
}

void RNUskfTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){

		xkAug = Matrix(SV_AUG, 1);
		PkAug = Matrix(SV_AUG, SV_AUG);
		XSigPred = Matrix(SV_AUG, SV_AUG_SIGMA);
		XSigPredAug = Matrix(SV_AUG, SV_AUG_SIGMA);

		b = gn->getb();
		B = gn->getB();
		Pxbk = Matrix(3, BV);

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
		gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		enableLocalization = true;

	} else{
		enableLocalization = false;
	}
}

void RNUskfTask::kill(){
	enableLocalization = false;
	RNRecurrentTask::kill();
}

void RNUskfTask::prediction(){

	double deltaDistance = 0.0, deltaAngle = 0.0;
	double deltaDistance_unbiased = 0.0, deltaAngle_unbiased = 0.0;
	gn->getIncrementPosition(&deltaDistance, &deltaAngle);

	if(deltaDistance * b(0,0) > 0 or std::abs(deltaDistance) >= std::abs(b(0,0))){
		deltaDistance_unbiased = deltaDistance + b(0,0); /*Corrige las medidas de la odometría con el vector bias calibrado*/
	}
	if(deltaAngle * b(1,0) > 0 or std::abs(deltaAngle) > std::abs(b(1,0))){
		deltaAngle_unbiased = deltaAngle + b(1,0);
	}

	xkAug(0, 0) = xk(0, 0);
	xkAug(1, 0) = xk(1, 0);
	xkAug(2, 0) = xk(2, 0);
	for(int i = 0; i < BV; i++){
		xkAug(SV + i, 0) = b(i, 0);;
	}

	for(int i = 0; i < SV; i++){
		for(int j= 0; j < SV; j++){
			PkAug(i, j) = Pk(i, j);
		}
	}

	for(int i = 0; i < SV; i++){
		for(int j= SV; j < SV+BV; j++){
			PkAug(i, j) = Pxbk(i, j-SV);
		}
	}

	for(int i = SV; i < SV+BV; i++){
		for(int j= 0; j < SV; j++){
			PkAug(i, j) = Pxbk(j, i-SV);
		}
	}

	for(int i = SV; i < SV+BV; i++){
		for(int j= SV; j < SV+BV; j++){
			PkAug(i, j) = B(i-SV, j-SV);
		}
	}

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
		//RNUtils::getOdometryPose(XSigPredAug.col(i), deltaDistance_unbiased, deltaAngle_unbiased, xk_1);
		//XSigPred.setCol(i, xk_1);
		XSigPred(0, i) = xk_1(0, 0);
		XSigPred(1, i) = xk_1(1, 0);
		XSigPred(2, i) = xk_1(2, 0);
		XSigPred(3, i) = XSigPredAug(3, i);
		XSigPred(4, i) = XSigPredAug(4, i);
		XSigPred(5, i) = XSigPredAug(5, i);
		XSigPred(6, i) = XSigPredAug(6, i);
		XSigPred(7, i) = XSigPredAug(7, i);
	}

	// Mean of state
	Matrix xkAug_mean(SV_AUG, 1);
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		xkAug_mean = xkAug_mean + wm[i] * XSigPred.col(i);
	}
	xkAug_mean(2, 0) = RNUtils::fixAngleRad(xkAug_mean(2, 0));
	xkAug_1 = xkAug_mean;

	//covariance matrix
	Matrix Pyy(SV_AUG, SV_AUG);
	for(int i = 0; i < SV_AUG_SIGMA; i++){
		Matrix diff = XSigPred.col(i) - xkAug_1;
		diff(2, 0) = RNUtils::fixAngleRad(diff(2, 0));
		Pyy = Pyy + wc[i] * (diff * ~diff);
	}
	PkAug = Pyy;

	Pk(0, 0) = PkAug(0, 0); 	Pk(0, 1) = PkAug(0, 1); 	Pk(0, 2) = PkAug(0, 2);
	Pk(1, 0) = PkAug(1, 0); 	Pk(1, 1) = PkAug(1, 1); 	Pk(1, 2) = PkAug(1, 2);
	Pk(2, 0) = PkAug(2, 0); 	Pk(2, 1) = PkAug(2, 1); 	Pk(2, 2) = PkAug(2, 2);
}

void RNUskfTask::obtainMeasurements(Matrix& zkli, std::vector<int>& ids){
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
			results(2*j, i) = zkl(j, 0);// + XSigPred(5, i);
			results(2*j + 1, i) = zkl(j, 1);// + XSigPred(6, i);
			if(not idsProcessed){
				ids.push_back((int)zkl(j, 3));
			}
			
		}

		for(int j = cameraIndex; j < totalLandmarks; j++){
			results(cameraIndex + j, i) = zkl(j, 1);// + XSigPred(7, i);
			if(not idsProcessed){
				ids.push_back((int)zkl(j, 3));
			}
		}

		idsProcessed = true;
	}

	zkli = results;	
}


void RNUskfTask::task(){
	if(enableLocalization){
		int rsize = 0, vsize = 0, rreject = 0, vreject = 0;
		int activeRL = 0, activeVL = 0;
		pk1 = Pk;
		pxbk1 = Pxbk;
		xk_1 = xk;
		xkAug_1 = xkAug;

		//printf("\n\n\nNew Iteration:\n");
		gn->getRobotPosition();
		prediction();

		//printf("P(k + 1|k)\n");
		//Pk.print();
		
		if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
			//printf("Error gordo...\n");
		}

		Matrix zkli;
		std::vector<int> ids;
		obtainMeasurements(zkli, ids);
		
		//printf("zkli:\n");
		//zkli.print();

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
		//printf("sizeH = %d\n", sizeH);
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
					zik_1(i + cameraIndex, 0) = lndmrk->getPointsYMean();

					currentR(cameraIndex + i, cameraIndex + i) = gn->getCameraAngleVariance();

					for(int j = 0; j < SV_AUG_SIGMA; j++){
						int zklIndex = RN_NONE;
						for(int k = 0; k < cameraLandmarksCount and (zklIndex == RN_NONE); k++){
							if(lndmrk->getMarkerId() == ids[k + laserLandmarksCount]){	
								zklIndex = k + 2 * laserLandmarksCount;
							}
						}
						if(lndmrk != NULL and (lndmrk->getMapId() == currentSector->getMapId()) and (lndmrk->getSectorId() == currentSector->getId()) and zklIndex != RN_NONE){				
							zi(cameraIndex + i, j) = zkli(zklIndex, j);
							//zl(cameraIndex + i, 0) = RNUtils::fixAngleRad(zl(cameraIndex + i, 0));
							//RNUtils::printLn("MapId: %d, SectorId: %d, markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {a: %f}", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(j + laserOffset, 0));
						}
					}

				}
			}

			//printf("zik_1:\n");
			//zik_1.print();

			//printf("zi:\n");
			//zi.print();

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

			Matrix Pxz(SV_AUG, sizeH);
			for(int i = 0; i < SV_AUG_SIGMA; i++){
				Matrix diff_x = XSigPred.col(i) - xkAug_1;
				diff_x(2, 0) = RNUtils::fixAngleRad(diff_x(2, 0));

				Matrix diff_z = zi.col(i) - zi_mean;
				Pxz = Pxz + wc[i] * (diff_x * ~diff_z);
			}

			Matrix Wk = Pxz * !Sk;

			Matrix Wxk(SV, sizeH);
			for(int i = 0; i < SV; i++){
				Wxk.setRow(i, Wk.row(i));
			}

			Matrix Wbk(BV, sizeH);

			for(int i = SV; i < SV_AUG; i++){
				Wbk.setRow(i-SV,  Wk.row(i));
			}

			//printf("Z(k + 1)\n");
			//zik_1.print();

			Matrix nu = zik_1 - zi_mean;
			for (int i = 0; i < activeRL; i++){
				double mdDistance = std::abs(nu(2 * i, 0) / std::sqrt(gn->getLaserDistanceVariance()));
				double mdAngle = std::abs(nu(2 * i + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));

				if (mdDistance > laserTMDistance or mdAngle > laserTMAngle){

					//RNUtils::printLn("RL landmark rejected...");
					nu(2 * i, 0) = 0.0;
					nu(2 * i + 1, 0) = 0.0;
					rsize--;
					rreject++;
				}
			}

			for(int i = 0; i < activeVL; i++){
				double mdAngle = std::abs(nu(cameraIndex + i, 0) / std::sqrt(gn->getCameraAngleVariance()));

				if (mdAngle > cameraTMAngle){
					//RNUtils::printLn("FL Landmark %d rejected...", ids[i + laserLandmarksCount]);
					nu(cameraIndex + i, 0) = 0;
					vsize--;
					vreject++;
				}
			}
			
			//printf("nu(k + 1)\n");
			//nu.print();

			xk = xk_1 + Wxk * nu;
			xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));

			xkAug(0, 0) = xk(0, 0);
			xkAug(1, 0) = xk(1, 0);
			xkAug(2, 0) = xk(2, 0);
			for(int i = 0; i < BV; i++){
				xkAug(SV + i, 0) = b(i, 0);;
			}


			//printf("X(k + 1|k + 1) postfix\n");
			//xk.print();

			char bufferpk1[256], bufferpk[256];
			sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			Pk = Pk - Wxk * Sk * ~Wxk;
			Pxbk = Pxbk - Wxk * Sk * ~Wbk;

			//printf("P(k + 1|k + 1)\n");
			//Pk.print();
			if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
				printf("Error gordo...\n");
			}
			sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));

			gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
			char buffer[1024];
			sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize, rreject, vreject);
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
			//gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
			char buffer[1024];
			sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize, rreject, vreject);
			if(test != NULL){
				fprintf(test, "%s", buffer);
			}
		}

		//xk.print();
		
		gn->getVisualLandmarks()->clear();
		gn->unlockVisualLandmarks();
		gn->unlockLaserLandmarks();
		

	} else {
		init();
	}
	RNUtils::sleep(20);
}

std::map<int, double> RNUskfTask::computeMahalanobis(const int& sigmaPoint, Matrix measure, const Matrix& zkli, const Matrix& xk, const Matrix& sr, const std::vector<int>& ids){
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


void RNUskfTask::predictMeasurements(Matrix& predictions, const Matrix& xk){
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

void RNUskfTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle){
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));
	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);
}

void RNUskfTask::OnMessageReceivedWithData(unsigned char* cad, int length){
	gn->lockVisualLandmarks();
	RNLandmarkList* markers = gn->getVisualLandmarks();
	markers->initializeFromString((char*)cad);
	gn->unlockVisualLandmarks();
}
