#include "RNKalmanLocalizationTask.h"

const double RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.014;

RNKalmanLocalizationTask::RNKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	test = std::fopen("laser_camera.txt","w+");
	this->startThread();
}

RNKalmanLocalizationTask::~RNKalmanLocalizationTask(){
	if(test != NULL){
		std::fclose(test);
	}
}

void RNKalmanLocalizationTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
		Pk = gn->getP();

		laserLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		cameraLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);


		laserTMDistance = gn->getLaserDistanceAlpha() / std::sqrt(gn->getLaserDistanceVariance());
		laserTMAngle = gn->getLaserAngleAlpha() / std::sqrt(gn->getLaserAngleVariance());
		//cameraTMDistance = gn->getCameraDistanceAlpha() / std::sqrt(gn->getCameraDistanceVariance());
		cameraTMDistance = 0.1;
		cameraTMAngle = gn->getCameraAngleAlpha() / std::sqrt(gn->getCameraAngleVariance());


		xk = gn->getRawEncoderPosition();
		xk.print();
		//gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		//gn->unlockLaserLandmarks();
		//gn->unlockVisualLandmarks();
		enableLocalization = true;

	} else{
		enableLocalization = false;
	}
}

void RNKalmanLocalizationTask::kill(){
	enableLocalization = false;
	RNRecurrentTask::kill();
}

void RNKalmanLocalizationTask::task(){
	if(enableLocalization){
		pk1 = Pk;
		xk_1 = xk;
		double deltaDistance = 0.0, deltaAngle = 0.0;
		gn->getIncrementPosition(&deltaDistance, &deltaAngle);

		Ak(0, 2) = -deltaDistance * std::sin(xk_1(2, 0) + deltaAngle/2.0);
		Ak(1, 2) = deltaDistance * std::cos(xk_1(2, 0) + deltaAngle/2.0);

		Bk(0, 0) = std::cos(xk_1(2, 0) + deltaAngle/2.0);
		Bk(0, 1) = -0.5 * deltaDistance * std::sin(xk_1(2, 0) + deltaAngle/2.0);

		Bk(1, 0) = std::sin(xk_1(2, 0) + deltaAngle/2.0);
		Bk(1, 1) = 0.5 * deltaDistance * std::cos(xk_1(2, 0) + deltaAngle/2.0);

		Bk(2, 0) = 0.0;
		Bk(2, 1) = 1.0;
		Matrix currentR = gn->getR();
		Matrix currentQ = gn->getQ();
		if(deltaDistance == 0.0 and deltaAngle == 0.0){
			currentQ = Matrix(2, 2);
		}
		RNUtils::getOdometryPose(xk, deltaDistance, deltaAngle, xk_1);

		Pk = (Ak * pk1 * ~Ak) + (Bk * currentQ * ~Bk);

		
		int totalLandmarks = 0;
		if(gn->isLaserSensorActivated()){
			totalLandmarks += laserLandmarksCount;
		}

		if(gn->isCameraSensorActivated()){
			totalLandmarks += cameraLandmarksCount;
		}

		int sizeH = (gn->isLaserSensorActivated() ? 2 * laserLandmarksCount : 0) + (gn->isCameraSensorActivated() ? cameraLandmarksCount : 0);
		Hk = Matrix(sizeH, STATE_VARIABLES);
		int laserIndex = 0, cameraIndex = 0;

		if(gn->isLaserSensorActivated()){
			cameraIndex = 2 * laserLandmarksCount;
		}
		for(int i = 0, zIndex = 0; i < gn->getCurrentSector()->landmarksSize(); i++){
			Matrix disp = Matrix(2, 1);
			double landmarkDistance;
			s_landmark* currLandmark = gn->getCurrentSector()->landmarkAt(i);
			if(gn->isLaserSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){
					zIndex = 2 * laserIndex;
					laserIndex++;

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, xk_1(0, 0), xk_1(1, 0));

					Hk(zIndex, 0) = -(currLandmark->xpos - xk_1(0, 0))/landmarkDistance;
					Hk(zIndex, 1) = -(currLandmark->ypos - xk_1(1, 0))/landmarkDistance;
					Hk(zIndex, 2) = 0.0;

					Hk(zIndex + 1, 0) = (currLandmark->ypos - xk_1(1, 0))/std::pow(landmarkDistance, 2);
					Hk(zIndex + 1, 1) = -(currLandmark->xpos - xk_1(0, 0))/std::pow(landmarkDistance, 2);
					Hk(zIndex + 1, 2) = -1.0;
				}
			}
			if(gn->isCameraSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
					zIndex = cameraIndex;
					cameraIndex++;
					double nrx, nry;
					RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk_1(2, 0), &nrx, &nry);
					disp(0, 0) = nrx;
					disp(1, 0) = nry;

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (xk_1(0, 0) + disp(0, 0)), (xk_1(1, 0) + disp(1, 0)));

					Hk(zIndex, 0) = (currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/std::pow(landmarkDistance, 2);
					Hk(zIndex, 1) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/std::pow(landmarkDistance, 2);
					Hk(zIndex, 2) = -1.0;
				}
			}
			disp(0, 0) = 0.0;
			disp(1, 0) = 0.0;
		}
		

		Matrix zkl;
		getObservations(zkl);
		
		Matrix zl(sizeH, 1);

		laserIndex = 0;
		cameraIndex = 0; 

		if(gn->isLaserSensorActivated()){
			cameraIndex = laserLandmarksCount;
		}

		
		int rsize = 0, vsize = 0;
		if(gn->isLaserSensorActivated()){
			gn->lockLaserLandmarks();
			rsize = gn->getLaserLandmarks()->size();
			for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				//RNUtils::printLn("Laser landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, float> > euclideanDistances;
				
				Matrix smallR(2, 2);
				smallR(0, 0) = gn->getLaserDistanceVariance();
				smallR(1, 1) = gn->getLaserAngleVariance();

				Matrix measure(2, 1);
				measure(0, 0) = lndmrk->getPointsXMean();
				measure(1, 0) = lndmrk->getPointsYMean();
				for (int j = laserIndex; j < cameraIndex; j++){
					
					Matrix smallHk(2, 3);
					smallHk(0, 0) = Hk(2 * j, 0);
					smallHk(0, 1) = Hk(2 * j, 1);
					smallHk(0, 2) = Hk(2 * j, 2);
					smallHk(1, 0) = Hk(2 * j + 1, 0);
					smallHk(1, 1) = Hk(2 * j + 1, 1);
					smallHk(1, 2) = Hk(2 * j + 1, 2);
					Matrix omega = smallHk * Pk * ~smallHk + smallR;
					Matrix obs(2, 1);
					obs(0, 0) = zkl(j, 0);
					obs(1, 0) = zkl(j, 1);
					Matrix mdk = ~(measure - obs) * omega * (measure - obs);
					float md = std::sqrt(mdk(0, 0));

					
					euclideanDistances.push_back(std::pair<int, float>(j, md));
					
				}

				float minorDistance = std::numeric_limits<float>::infinity();
				int indexFound = RN_NONE;
				for (int j = 0; j < euclideanDistances.size(); j++){
					if(euclideanDistances.at(j).second < minorDistance){
						minorDistance = euclideanDistances.at(j).second;
						indexFound = euclideanDistances.at(j).first;
					}	
				}
				
				//RNUtils::printLn("Matched landmark: {idx : %d, MHD: %f}", indexFound, minorDistance);
				if(indexFound > RN_NONE){
					zl(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl(indexFound, 0);
					zl(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl(indexFound, 1);
					if(zl(2 * indexFound + 1, 0) > M_PI){
						zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) - 2 * M_PI;
					} else if(zl(2 * indexFound + 1, 0) < -M_PI){
						zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) + 2 * M_PI;
					}
					//RNUtils::printLn("markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {d: %f, a: %f}", indexFound, lndmrk->getPointsXMean() , lndmrk->getPointsYMean() , zkl(indexFound , 0), zkl(indexFound , 1), zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));
					
					double mdDistance = std::abs(zl(2 * indexFound, 0) / std::sqrt(gn->getLaserDistanceVariance()));
					double mdAngle = std::abs(zl(2 * indexFound + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));

					if (mdDistance > laserTMDistance or mdAngle > laserTMAngle){
	
						//RNUtils::printLn("landmark %d rejected...", indexFound);
						zl(2 * indexFound, 0) = 0.0;
						zl(2 * indexFound + 1, 0) = 0.0;
						rsize--;
					}
				}
			}
			gn->unlockLaserLandmarks();
		}

		if(gn->isCameraSensorActivated()){
			gn->lockVisualLandmarks();
			int laserOffset = gn->isLaserSensorActivated() ? laserLandmarksCount : 0;

			vsize = gn->getVisualLandmarks()->size();
			for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
				//RNUtils::printLn("Id: {M: %d, S: %d, L:%d} - angle: %lf", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), lndmrk->getPointsYMean());
				bool validQR = true;		

				if (lndmrk->getMapId() > RN_NONE && lndmrk->getSectorId() > RN_NONE && lndmrk->getMarkerId() > RN_NONE){
					//Compares the current landmark with the remaining landmarks
					for (int j = 0; j < gn->getVisualLandmarks()->size() and (validQR); j++){
						if (j != i){
							if (lndmrk->getMarkerId() == gn->getVisualLandmarks()->at(j)->getMarkerId()){
								validQR = false;
								//RNUtils::printLn("Current landmark doesn't have a unique ID. Applying Mahalanobis distance...");
							}
						}
					}
				} else {
					validQR = false;
				}

				//If the current landmark has a unique id, checks if there's a match with the database
				if (validQR){
					//for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
					for (int j = cameraIndex; j < zkl.rows_size(); j++){

						if((lndmrk->getMapId() == gn->getCurrentSector()->getMapId()) and (lndmrk->getSectorId() == gn->getCurrentSector()->getId()) and (lndmrk->getMarkerId() == ((int)zkl(j, 3)))){
							//double distanceFixed = (zkl(j, 2) - gn->getRobotHeight()) * std::tan(extraParameter->second);
							double distanceFixed = lndmrk->getPointsXMean();
							double angleFixed = lndmrk->getPointsYMean();
							

							zl(j + laserOffset, 0) = angleFixed - zkl(j, 1);
							if(zl(j + laserOffset, 0) > M_PI){
								zl(j + laserOffset, 0) = zl(j + laserOffset, 0) - 2 * M_PI;
							} else if(zl(j + laserOffset, 0) < -M_PI){
								zl(j + laserOffset, 0) = zl(j + laserOffset, 0) + 2 * M_PI;
							}
							//RNUtils::printLn("MapId: %d, SectorId: %d, markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {a: %f}", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(j + laserOffset, 0));
						}
					}
				}
			}
			//Calculates Mahalanobis distance to determine whether a measurement is acceptable.				
			for (int i = cameraIndex; i < zkl.rows_size(); i++){
				
				double mdAngle = std::abs(zl(i + laserOffset, 0) / std::sqrt(gn->getCameraAngleVariance()));

				if (mdAngle > cameraTMAngle){
					//RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
					zl(i + laserOffset, 0) = 0;
					vsize--;
				}
			}	
			gn->getVisualLandmarks()->clear();
			gn->unlockVisualLandmarks();

		}

		Matrix Sk = Hk * Pk * ~Hk + currentR;
		Matrix Wk = Pk * ~Hk * !Sk;


		Wk = fixFilterGain(Wk);

		//printf("Wk:\n");
		//Wk.print();

		char bufferpk1[256], bufferpk[256];
		sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
		xk = xk_1 + Wk * zl;
		//printf("xk:\n");
		//xk.print();
		xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));
		gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		char buffer[1024];
		sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize);
		if(test != NULL){
			fprintf(test, "%s", buffer);
		}
		
		
		//gn->setPosition(newPosition(0, 0), newPosition(1, 0), newPosition(2, 0));
		//bool isInsidePolygon = gn->getCurrentSector()->checkPointXYInPolygon(PointXY(xk(0, 0), xk(1, 0)));
		//if(not isInsidePolygon){
			//gn->loadSector(gn->getCurrenMapId(), gn->getNextSectorId());
			//RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", gn->getNextSectorId(), gn->getCurrentSector()->getName().c_str());
			//gn->setNextSectorId(RN_NONE);
			//new position
			//gn->setLastVisitedNode(0);
	        //gn->setPosition(newPosition(0, 0) + gn->getNextSectorCoord().getX(), newPosition(1, 0) + gn->getNextSectorCoord().getY(), newPosition(2, 0));
	        //init();
		//}
	} else {
		init();
	}
}

Matrix RNKalmanLocalizationTask::fixFilterGain(const Matrix wk){
	Matrix result = wk;
	for(int i = 0; i < wk.rows_size(); i++){
		for(int j = 0; j < wk.cols_size(); j++){
			if(wk(i, j) >= -1.0e-5 and wk(i, j) <= 1.0e-5){
				result(i, j) = 0.0;
			}
		}
	}
	return result;
}


void RNKalmanLocalizationTask::getObservations(Matrix& observations){
	int totalLandmarks = 0;
	int laserIndex = 0, cameraIndex = 0;
	if(gn->isLaserSensorActivated()){
		totalLandmarks += laserLandmarksCount;
		cameraIndex += laserLandmarksCount;	
	}

	if(gn->isCameraSensorActivated()){
		totalLandmarks += cameraLandmarksCount;

	}


	Matrix result(totalLandmarks, 4);

	for(int k = 0; k < gn->getCurrentSector()->landmarksSize(); k++){
		int zIndex = RN_NONE;
		double distance = 0, angle = 0;
		Matrix disp = Matrix(2, 1);
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);
		if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
			double nrx, nry;
			RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk_1(2, 0), &nrx, &nry);
			disp(0, 0) = nrx;
			disp(1, 0) = nry;
		}
		landmarkObservation(xk_1, disp, landmark, distance, angle);

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
			result(zIndex, 0) = distance;
			if(angle > M_PI){
				angle = angle - 2 * M_PI;
			} else if(angle < -M_PI){
				angle = angle + 2 * M_PI;
			}

			result(zIndex, 1) = angle;
			result(zIndex, 2) = landmark->zpos;
			result(zIndex, 3) = (double)landmark->id;
		}
		
	}

	observations = result;
}

void RNKalmanLocalizationTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle){
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));
	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);
}

void RNKalmanLocalizationTask::OnMessageReceivedWithData(unsigned char* cad, int length){
	gn->lockVisualLandmarks();
	RNLandmarkList* markers = gn->getVisualLandmarks();
	markers->initializeFromString((char*)cad);
	gn->unlockVisualLandmarks();
}