#include "RNKalmanLocalizationTask.h"

const double RNKalmanLocalizationTask::MAX_LASER_DISTANCE_ERROR = 0.05;
const double RNKalmanLocalizationTask::MAX_LASER_ANGLE_ERROR = 0.07;
const double RNKalmanLocalizationTask::MAX_CAMERA_DISTANCE_ERROR = 0.2;
const double RNKalmanLocalizationTask::MAX_CAMERA_ANGLE_ERROR = 0.1;

const double RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.2695;
const double RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.0109;

RNKalmanLocalizationTask::RNKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description){
	enableLocalization = false;
	test = std::fopen("solo_camera.txt","w+");
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
		rfidLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);

		laserTMDistance = MAX_LASER_DISTANCE_ERROR / std::sqrt(gn->getLaserDistanceVariance());
		laserTMAngle = MAX_LASER_ANGLE_ERROR / std::sqrt(gn->getLaserAngleVariance());
		cameraTMDistance = MAX_CAMERA_DISTANCE_ERROR / std::sqrt(gn->getCameraDistanceVariance());
		cameraTMAngle = MAX_CAMERA_ANGLE_ERROR / std::sqrt(gn->getCameraAngleVariance());
		rfidTMDistance = 0.2;
		rfidTMAngle = 0.2;

		xk = gn->getRawEncoderPosition();
		xk.print();
		gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		gn->unlockLaserLandmarks();
		gn->unlockVisualLandmarks();
		enableLocalization = true;

	} else{
		enableLocalization = false;
	}
}

void RNKalmanLocalizationTask::onKilled(){
	enableLocalization = false;
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

		if(gn->isRfidSensorActivated()){
			totalLandmarks += rfidLandmarksCount;
		}

		Hk = Matrix(2 * totalLandmarks, STATE_VARIABLES);
		int laserIndex = 0, cameraIndex = 0, rfidIndex = 0;
		if(gn->isLaserSensorActivated()){
			cameraIndex += laserLandmarksCount;
			rfidIndex += laserLandmarksCount;
		}
		if(gn->isCameraSensorActivated()){
			rfidIndex += cameraLandmarksCount;
		}
		for(int i = 0, zIndex = 0; i < gn->getCurrentSector()->landmarksSize(); i++){
			if(gn->isLaserSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){
					zIndex = 2 * laserIndex;
					laserIndex++;
				}
			}
			if(gn->isCameraSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
					zIndex = 2 * cameraIndex;
					cameraIndex++;
				}
			}
			if(gn->isRfidSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_RFID_STR){
					zIndex = 2 * rfidIndex;
					rfidIndex++;
				}
			}
			s_landmark* currLandmark = gn->getCurrentSector()->landmarkAt(i);
			double landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, xk_1(0, 0), xk_1(1, 0));

			Hk(zIndex, 0) = -(currLandmark->xpos - xk_1(0, 0))/landmarkDistance;
			Hk(zIndex, 1) = -(currLandmark->ypos - xk_1(1, 0))/landmarkDistance;
			Hk(zIndex, 2) = 0.0;

			Hk(zIndex + 1, 0) = (currLandmark->ypos - xk_1(1, 0))/landmarkDistance;
			Hk(zIndex + 1, 1) = -(currLandmark->xpos - xk_1(0, 0))/landmarkDistance;
			Hk(zIndex + 1, 2) = -1.0;
		}

		Matrix zkl;
		getObservations(zkl);
		
		Matrix zl(2 * totalLandmarks, 1);

		laserIndex = 0;
		cameraIndex = 0; 
		rfidIndex = 0;

		if(gn->isLaserSensorActivated()){
			cameraIndex += laserLandmarksCount;
			rfidIndex += laserLandmarksCount;
		}
		if(gn->isCameraSensorActivated()){
			rfidIndex += cameraLandmarksCount;
		}
		int rsize = 0, vsize = 0;
		if(gn->isLaserSensorActivated()){
			gn->lockLaserLandmarks();
			rsize = gn->getLaserLandmarks()->size();
			for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				//RNUtils::printLn("Laser landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, float> > euclideanDistances;
				for (int j = laserIndex; j < cameraIndex; j++){
					float ed = std::sqrt(std::pow(zkl(j, 0), 2.0) + std::pow(lndmrk->getPointsXMean(), 2.0) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					//ed = ed / std::sqrt(gn->getCameraAngleVariance());
					euclideanDistances.push_back(std::pair<int, float>(j, ed));
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
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
					
					double mdDistance = std::abs(zl(2 * indexFound, 0) / std::sqrt(gn->getLaserDistanceVariance()));
					double mdAngle = std::abs(zl(2 * indexFound + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));
					//printf("Mhd: (%d), %lg, %lg, nud: %lg, nua: %lg\n", i, mdDistance, mdAngle, zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));
					if (mdDistance > laserTMDistance and mdAngle > laserTMAngle){
						//distanceThreshold = (máxima zl que permite un buen matching)/sigma
						RNUtils::printLn("landmark %d rejected...", indexFound);
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
			vsize = gn->getVisualLandmarks()->size();
			for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
				//RNUtils::printLn("Id: {M: %d, S: %d, L:%d} - angle: %lf", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), lndmrk->getPointsYMean());

				int mapId = lndmrk->getMapId();
				int sectorId = lndmrk->getSectorId();
				int markerId = lndmrk->getMarkerId();
				//std::pair<std::string, double>* extraParameter = lndmrk->getExtraParameter(OPTICAL_THETA_STR);
				bool validQR = true;		

				if (mapId > RN_NONE && sectorId > RN_NONE && markerId > RN_NONE){
					//Compares the current landmark with the remaining landmarks
					for (int j = 0; j < gn->getVisualLandmarks()->size() and (validQR); j++){
						if (j != i){
							if (markerId == gn->getVisualLandmarks()->at(j)->getMarkerId()){
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
					for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
						if(markerId == ((int)zkl(j, 3))){
							//double distanceFixed = (zkl(j, 2) - gn->getRobotHeight()) * std::tan(extraParameter->second);
							double distanceFixed = lndmrk->getPointsXMean();
							double angleFixed = lndmrk->getPointsYMean();
							//RNUtils::printLn("Marker original angle: %lf", angleFixed);
							/*double xr = CAMERA_ERROR_POSITION_X + distanceFixed * std::cos(angleFixed);
							double yr = CAMERA_ERROR_POSITION_Y + distanceFixed * std::sin(angleFixed);
							distanceFixed = std::sqrt(std::pow(xr, 2.0) + std::pow(yr, 2.0));
							angleFixed = angleFixed + std::atan2(-yr, -xr);*/
				
							//zl(2 * j, 0) = distanceFixed - zkl(j, 0);
							zl(2 * j, 0) = 0.0;
							/*if(distanceFixed > 1.0){
								currentR(2 * j, 2 * j) = currentR(2 * j, 2 * j) * std::exp(std::abs(distanceFixed - zkl(j, 0)));
							}*/
							zl(2 * j + 1, 0) = angleFixed - zkl(j, 1);
							if(zl(2 * j + 1, 0) > M_PI){
								zl(2 * j + 1, 0) = zl(2 * j + 1, 0) - 2 * M_PI;
							} else if(zl(2 * j + 1, 0) < -M_PI){
								zl(2 * j + 1, 0) = zl(2 * j + 1, 0) + 2 * M_PI;
							}
							RNUtils::printLn("markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {d: %f, a: %f}", markerId, distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(2 * j, 0), zl(2 * j + 1, 0));
						}
					}
				} else { 
					std::vector<std::pair<int, double> > cameraDistances;
					for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
						double cd = std::sqrt(std::pow(zkl(j, 0), 2.0) + std::pow(lndmrk->getPointsXMean(), 2.0) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
						
						cameraDistances.push_back(std::pair<int, double>(j, cd));
						//RNUtils::printLn("Mahalanobis visual Distance[%d]: %f for %f rad", j, cd, zkl(j, 1));
					}

					double minorDifference = std::numeric_limits<double>::infinity();
					int indexFound = RN_NONE;
					for (int j = 0; j < cameraDistances.size(); j++){
						if(cameraDistances.at(j).second < minorDifference){
							minorDifference = cameraDistances.at(j).second;
							indexFound = cameraDistances.at(j).first;
						}
					}

					if(indexFound > RN_NONE){
						//double distanceFixed = (zkl(j, 2) - gn->getRobotHeight()) * std::tan(extraParameter->second);
						double distanceFixed = lndmrk->getPointsXMean();
						double angleFixed = lndmrk->getPointsYMean();
						/*double xr = CAMERA_ERROR_POSITION_X + distanceFixed * std::cos(angleFixed);
						double yr = CAMERA_ERROR_POSITION_Y + distanceFixed * std::sin(angleFixed);
						distanceFixed = std::sqrt(std::pow(xr, 2) + std::pow(yr, 2));
						angleFixed = std::atan2(yr, xr);*/
						if(distanceFixed > 1.0){
							currentR(2 * indexFound, 2 * indexFound) = currentR(2 * indexFound, 2 * indexFound) * std::exp(std::abs(distanceFixed - zkl(indexFound, 0)));
						}
						
						//zl(2 * indexFound, 0) = distanceFixed - zkl(indexFound, 0);
						zl(2 * indexFound, 0) = 0.0;
						zl(2 * indexFound + 1, 0) = angleFixed - zkl(indexFound, 1);
						if(zl(2 * indexFound + 1, 0) > M_PI){
							zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) - 2 * M_PI;
						} else if(zl(2 * indexFound + 1, 0) < -M_PI){
							zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) + 2 * M_PI;
						}
					}

				}
			}
			//Calculates Mahalanobis distance to determine whether a measurement is acceptable.				
			for (int i = cameraIndex; i < (cameraIndex + cameraLandmarksCount); i++){

				double mdDistance = std::abs(zl(2 * i, 0) / std::sqrt(gn->getCameraDistanceVariance()));
				double mdAngle = std::abs(zl(2 * i + 1, 0) / std::sqrt(gn->getCameraAngleVariance()));
				//printf("Mh: (%d): %g. nu: %g\n", i, mahalanobisDistance, zl(2 * i + 1, 0));
				if (mdDistance > cameraTMDistance or mdAngle > cameraTMAngle){
					RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
					//distanceThreshold = (máxima zl que permite un buen matching)/sigma
					zl(2 * i, 0) = 0;
					zl(2 * i + 1, 0) = 0;
					vsize--;
				}
			}	
			gn->getVisualLandmarks()->clear();
			gn->unlockVisualLandmarks();
			//RNUtils::printLn("-----------> Zl, Variance: %g", gn->getCameraAngleVariance());
			//zl.print();
		}

		/*if(gn->isRfidSensorActivated()){
			gn->lockRFIDLandmarks();
			for (int i = 0; i < gn->getRFIDLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getRFIDLandmarks()->at(i);

				//RNUtils::printLn("landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, double> > euclideanDistances;
				for (int j = rfidIndex; j < (rfidIndex + rfidLandmarksCount); j++){
					double ed = std::sqrt(std::pow(zkl(j, 0), 2) + std::pow(lndmrk->getPointsXMean(), 2) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					euclideanDistances.push_back(std::pair<int, double>(j, ed));
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
				}

				double minorDistance = std::numeric_limits<double>::infinity();
				int indexFound = RN_NONE;
				for (int j = 0; j < euclideanDistances.size(); j++){
					if(euclideanDistances.at(j).second < rfidTMDistance){
						minorDistance = euclideanDistances.at(j).second;
						indexFound = euclideanDistances.at(j).first;
					}	
				}
				
				//RNUtils::printLn("Matched landmark: {idx : %d, MHD: %f}\n", indexFound, minorDistance);
				if(indexFound > RN_NONE){
					zl(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl(indexFound, 0);
					zl(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl(indexFound, 1);
				}
			}
			gn->unlockRFIDLandmarks();
		}*/
		//zl.print();
		Matrix Sk = Hk * Pk * ~Hk + currentR;
		Matrix Wk = Pk * ~Hk * !Sk;
		char bufferpk1[256], bufferpk[256];
		sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
		xk = xk_1 + Wk * zl;
		gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		char buffer[1024];
		sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize);
		if(test != NULL){
			fprintf(test, "%s", buffer);
		}
		
		
		//gn->setPosition(newPosition(0, 0), newPosition(1, 0), newPosition(2, 0));
		double angle = 0.0;
		bool isInsidePolygon = gn->getCurrentSector()->checkPointXYInPolygon(PointXY(xk(0, 0), xk(1, 0)), angle);

		/*if(not isInsidePolygon){
			gn->loadSector(gn->getCurrenMapId(), gn->getNextSectorId());
			//RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", gn->getNextSectorId(), gn->getCurrentSector()->getName().c_str());
			gn->setNextSectorId(RN_NONE);
			//new position
			gn->setLastVisitedNode(0);
	        gn->setPosition(newPosition(0, 0) + gn->getNextSectorCoord().getX(), newPosition(1, 0) + gn->getNextSectorCoord().getY(), newPosition(2, 0));
	        init();
		}*/
	} else {
		init();
	}
}

void RNKalmanLocalizationTask::getObservations(Matrix& observations){
	int totalLandmarks = 0;
	int laserIndex = 0, cameraIndex = 0, rfidIndex = 0;
	if(gn->isLaserSensorActivated()){
		totalLandmarks += laserLandmarksCount;
		cameraIndex += laserLandmarksCount;
		rfidIndex += laserLandmarksCount;
	}

	if(gn->isCameraSensorActivated()){
		totalLandmarks += cameraLandmarksCount;
		rfidIndex += cameraLandmarksCount;
	}

	if(gn->isRfidSensorActivated()){
		totalLandmarks += rfidLandmarksCount;
	}

	Matrix result(totalLandmarks, 4);

	for(int k = 0, zIndex = 0; k < gn->getCurrentSector()->landmarksSize(); k++){

		double distance = 0, angle = 0;
		Matrix disp = Matrix(2, 1);
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);
		if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
			disp(0, 0) = CAMERA_ERROR_POSITION_X;
			disp(1, 0) = CAMERA_ERROR_POSITION_Y;
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
		if(gn->isRfidSensorActivated()){
			if(landmark->type == XML_SENSOR_TYPE_RFID_STR){
				zIndex = rfidIndex;
				rfidIndex++;
			}
		}

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

	observations = result;
}

void RNKalmanLocalizationTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle){
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));
	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);
}