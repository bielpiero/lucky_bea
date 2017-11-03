#include "RNKalmanLocalizationTask.h"

const float RNKalmanLocalizationTask::MAX_LASER_DISTANCE_ERROR = 0.07;
const float RNKalmanLocalizationTask::MAX_LASER_ANGLE_ERROR = 0.025;
const float RNKalmanLocalizationTask::MAX_CAMERA_DISTANCE_ERROR = 0.4;
const float RNKalmanLocalizationTask::MAX_CAMERA_ANGLE_ERROR = 0.1;

const float RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.2695;
const float RNKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.0109;

RNKalmanLocalizationTask::RNKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description){
	enableLocalization = false;
	test = std::fopen("localization-data.txt","w+");
}

RNKalmanLocalizationTask::~RNKalmanLocalizationTask(){
	if(test!= NULL){
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
		enableLocalization = true;

		gn->unlockLaserLandmarks();
		gn->unlockVisualLandmarks();

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
		Ak(0, 2) = -gn->getRawDeltaPosition()(0, 0) * std::sin(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);
		Ak(1, 2) = gn->getRawDeltaPosition()(0, 0) * std::cos(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);

		Bk(0, 0) = std::cos(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);
		Bk(0, 1) = -0.5 * gn->getRawDeltaPosition()(0, 0) * std::sin(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);

		Bk(1, 0) = std::sin(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);
		Bk(1, 1) = 0.5 * gn->getRawDeltaPosition()(0, 0) * std::cos(gn->getRawEncoderPosition()(2, 0) + gn->getRawDeltaPosition()(1, 0)/2);

		Bk(2, 0) = 0.0;
		Bk(2, 1) = 1.0;

		Matrix currentQ = gn->getQ();
		if(gn->getRawDeltaPosition()(0, 0) == 0.0 and gn->getRawDeltaPosition()(1, 0) == 0.0){
			currentQ = Matrix(2, 2);
		}

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
			Hk(zIndex, 0) = -((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0))/std::sqrt(std::pow((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0), 2) + std::pow((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0), 2));
			Hk(zIndex, 1) = -((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0))/std::sqrt(std::pow((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0), 2) + std::pow((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0), 2));
			Hk(zIndex, 2) = 0.0;

			Hk(zIndex + 1, 0) = ((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0))/(std::pow((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0), 2) + std::pow((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0), 2));
			Hk(zIndex + 1, 1) = -((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0))/(std::pow((gn->getCurrentSector()->landmarkAt(i)->xpos) - gn->getRawEncoderPosition()(0, 0), 2) + std::pow((gn->getCurrentSector()->landmarkAt(i)->ypos) - gn->getRawEncoderPosition()(1, 0), 2));
			Hk(zIndex + 1, 2) = -1.0;
		}

		Matrix zkl;
		getObservations(zkl);
		Matrix Sk = Hk * Pk * ~Hk + gn->getR();
		Matrix Wk = Pk * ~Hk * !Sk;
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
		
		if(gn->isLaserSensorActivated()){
			gn->lockLaserLandmarks();	
			for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				//RNUtils::printLn("Laser landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, float> > euclideanDistances;
				for (int j = laserIndex; j < cameraIndex; j++){
					float ed = std::sqrt(std::pow(zkl(j, 0), 2) + std::pow(lndmrk->getPointsXMean(), 2) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					//ed = ed / std::sqrt(gn->getCameraAngleVariance());
					euclideanDistances.push_back(std::pair<int, float>(j, ed));
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
				}

				float minorDistance = std::numeric_limits<float>::infinity();
				int indexFound = RN_NONE;
				for (int j = 0; j < euclideanDistances.size(); j++){
					if(euclideanDistances.at(j).second < laserTMDistance){
						minorDistance = euclideanDistances.at(j).second;
						indexFound = euclideanDistances.at(j).first;
					}	
				}
				
				//RNUtils::printLn("Matched landmark: {idx : %d, MHD: %f}\n", indexFound, minorDistance);
				if(indexFound > RN_NONE){
					zl(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl(indexFound, 0);
					zl(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl(indexFound, 1);
					
					float mdDistance = std::abs(zl(2 * indexFound, 0) / std::sqrt(gn->getLaserDistanceVariance()));
					float mdAngle = std::abs(zl(2 * indexFound + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));
					//printf("Mh: (%d): %g. nu: %g\n", i, mahalanobisDistance, zl(2 * i + 1, 0));
					if (mdDistance > laserTMDistance and mdAngle > laserTMAngle){
						//distanceThreshold = (máxima zl que permite un buen matching)/sigma
						zl(2 * i, 0) = 0;
						zl(2 * i + 1, 0) = 0;
					}
					
				}
			}
			gn->unlockLaserLandmarks();
		}

		if(gn->isCameraSensorActivated()){
			gn->lockVisualLandmarks();

			for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
				//RNUtils::printLn("Id: {M: %d, S: %d, L:%d} - angle: %lf", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), lndmrk->getPointsYMean());

				int mapId = lndmrk->getMapId();
				int sectorId = lndmrk->getSectorId();
				int markerId = lndmrk->getMarkerId();
				std::pair<std::string, float>* extraParameter = lndmrk->getExtraParameter(OPTICAL_THETA_STR);
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
							double distanceFixed = (zkl(j, 2) - gn->getRobotHeight()) * std::tan(extraParameter->second);
							double angleFixed = lndmrk->getPointsYMean();
							
							double xr = CAMERA_ERROR_POSITION_X + distanceFixed * std::cos(angleFixed);
							double yr = CAMERA_ERROR_POSITION_Y + distanceFixed * std::sin(angleFixed);
							distanceFixed = std::sqrt(std::pow(xr, 2) + std::pow(yr, 2));
							angleFixed = std::atan2(yr, xr);
				
							zl(2 * j, 0) = distanceFixed - zkl(j, 0);
							zl(2 * j + 1, 0) = angleFixed - zkl(j, 1);
							RNUtils::printLn("markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {d: %f, a: %f}", markerId, distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(2 * j, 0), zl(2 * j + 1, 0));
						}
					}
				} else { 
					std::vector<std::pair<int, float> > cameraDistances;
					for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
						double distanceApprox = std::tan(extraParameter->second);
						float cd = std::sqrt(std::pow(distanceApprox - zkl(j, 0), 2) + std::pow(lndmrk->getPointsYMean() - zkl(j, 1), 2)); 
						cameraDistances.push_back(std::pair<int, float>(j, cd));
						//RNUtils::printLn("Mahalanobis visual Distance[%d]: %f for %f rad", j, cd, zkl(j, 1));
					}

					float minorDifference = std::numeric_limits<float>::infinity();
					int indexFound = RN_NONE;
					for (int j = 0; j < cameraDistances.size(); j++){
						if(cameraDistances.at(j).second < minorDifference){
							minorDifference = cameraDistances.at(j).second;
							indexFound = cameraDistances.at(j).first;
						}
					}

					if(indexFound > RN_NONE){
						double distanceFixed = (zkl(indexFound, 2) - gn->getRobotHeight()) * std::tan(extraParameter->second);
						double angleFixed = lndmrk->getPointsYMean();
						double xr = CAMERA_ERROR_POSITION_X + distanceFixed * std::cos(angleFixed);
						double yr = CAMERA_ERROR_POSITION_Y + distanceFixed * std::sin(angleFixed);
						distanceFixed = std::sqrt(std::pow(xr, 2) + std::pow(yr, 2));
						angleFixed = std::atan2(yr, xr);
						
						zl(2 * indexFound, 0) = distanceFixed - zkl(indexFound, 0);
						zl(2 * indexFound + 1, 0) = angleFixed - zkl(indexFound, 1);
					}

				}
			}
			//Calculates Mahalanobis distance to determine whether a measurement is acceptable.				
			for (int i = cameraIndex; i < (cameraIndex + cameraLandmarksCount); i++){
				float mdDistance = std::abs(zl(2 * i, 0) / std::sqrt(gn->getCameraDistanceVariance()));
				float mdAngle = std::abs(zl(2 * i + 1, 0) / std::sqrt(gn->getCameraAngleVariance()));
				//printf("Mh: (%d): %g. nu: %g\n", i, mahalanobisDistance, zl(2 * i + 1, 0));
				if (mdDistance > cameraTMDistance or mdAngle > cameraTMAngle){
					RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
					//distanceThreshold = (máxima zl que permite un buen matching)/sigma
					zl(2 * i, 0) = 0;
					zl(2 * i + 1, 0) = 0;
				}
			}			
			gn->unlockVisualLandmarks();
			//RNUtils::printLn("-----------> Zl, Variance: %g", gn->getCameraAngleVariance());
			//zl.print();
		}

		if(gn->isRfidSensorActivated()){
			gn->lockRFIDLandmarks();
			for (int i = 0; i < gn->getRFIDLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getRFIDLandmarks()->at(i);

				//RNUtils::printLn("landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, float> > euclideanDistances;
				for (int j = rfidIndex; j < (rfidIndex + rfidLandmarksCount); j++){
					float ed = std::sqrt(std::pow(zkl(j, 0), 2) + std::pow(lndmrk->getPointsXMean(), 2) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					euclideanDistances.push_back(std::pair<int, float>(j, ed));
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
				}

				float minorDistance = std::numeric_limits<float>::infinity();
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
		}
		//zl.print();
		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		Matrix newPosition = gn->getRawEncoderPosition() + Wk * zl;
		
		gn->setPosition(newPosition(0, 0), newPosition(1, 0), newPosition(2, 0));
		float angle = 0.0;
		bool isInsidePolygon = gn->getCurrentSector()->checkPointXYInPolygon(PointXY(newPosition(0, 0), newPosition(1, 0)), angle);

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

	RNUtils::sleep(10);
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

		float distance = 0, angle = 0;
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);
		landmarkObservation(gn->getRawEncoderPosition(), landmark, distance, angle);

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
		result(zIndex, 3) = (float)landmark->id;
	}

	observations = result;
}

void RNKalmanLocalizationTask::landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle){
	distance = std::sqrt(std::pow(landmark->xpos - Xk(0, 0), 2) + std::pow(landmark->ypos - Xk(1, 0), 2));
	angle = std::atan2(landmark->ypos - Xk(1, 0), landmark->xpos - Xk(0, 0)) - Xk(2, 0);
}