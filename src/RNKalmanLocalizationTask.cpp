#include "RNKalmanLocalizationTask.h"

RNKalmanLocalizationTask::RNKalmanLocalizationTask(const char* name, const char* description) : RNLocalizationTask(name, description){
	enableLocalization = false;
}

RNKalmanLocalizationTask::~RNKalmanLocalizationTask(){

}

void RNKalmanLocalizationTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
		Pk = gn->getP();

		laserLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		cameraLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);
		rfidLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);

		laserThresholdMatching = 0.2;
		cameraThresholdMatching = 7.651107;
		rfidThresholdMatching = 0.2;
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
		int laserIndex = 0, cameraIndex = laserLandmarksCount, rfidIndex = (laserLandmarksCount + cameraLandmarksCount);
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
		cameraIndex = laserLandmarksCount;
		rfidIndex = (laserLandmarksCount + cameraLandmarksCount);
		
		if(gn->isLaserSensorActivated()){
			gn->lockLaserLandmarks();	
			for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				//RNUtils::printLn("Laser landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
				std::vector<std::pair<int, float> > euclideanDistances;
				for (int j = laserIndex; j < cameraIndex; j++){
					float ed = std::sqrt(std::pow(zkl(j, 0), 2) + std::pow(lndmrk->getPointsXMean(), 2) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					euclideanDistances.push_back(std::pair<int, float>(j, ed));
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
				}

				float minorDistance = std::numeric_limits<float>::infinity();
				int indexFound = RN_NONE;
				for (int j = 0; j < euclideanDistances.size(); j++){
					if(euclideanDistances.at(j).second < laserThresholdMatching){
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

				bool validQR = true;		

				if (mapId > 0 && sectorId > 0 && markerId > 0){
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
						if(markerId == ((int)zkl(j, 2))){
							zl(2 * j, 0) = 0;
							zl(2 * j + 1, 0) = std::abs(lndmrk->getPointsYMean()) - std::abs(zkl(j, 1));
						}
					}
				} else { 
					std::vector<std::pair<int, float> > cameraAngleDifferences;
					for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
						float cd = std::abs((lndmrk->getPointsYMean() - zkl(j, 1))); /*/ std::sqrt(gn->getCameraAngleVariance()))*/
						cameraAngleDifferences.push_back(std::pair<int, float>(j, cd));
						//RNUtils::printLn("Mahalanobis visual Distance[%d]: %f for %f rad", j, cd, zkl(j, 1));
					}

					float minorDifference = std::numeric_limits<float>::infinity();
					int indexFound = RN_NONE;
					for (int j = 0; j < cameraAngleDifferences.size(); j++){
						if(cameraAngleDifferences.at(j).second < minorDifference){
							minorDifference = cameraAngleDifferences.at(j).second;
							indexFound = cameraAngleDifferences.at(j).first;
						}
					}

					if(indexFound > RN_NONE){
						zl(2 * indexFound, 0) = 0;
						zl(2 * indexFound + 1, 0) = std::abs(lndmrk->getPointsYMean()) - std::abs(zkl(indexFound, 1));

					}

				}
			}
			//Calculates Mahalanobis distance to determine whether a measurement is acceptable.				
			for (int i = cameraIndex; i < (cameraIndex + cameraLandmarksCount); i++){
				float mahalanobisDistance = std::abs(zl(2 * i + 1, 0) / std::sqrt(gn->getCameraAngleVariance()));
				//printf("Mh: (%d): %g. nu: %g\n", i, mahalanobisDistance, zl(2 * i + 1, 0));
				if (mahalanobisDistance > cameraThresholdMatching){
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
					if(euclideanDistances.at(j).second < rfidThresholdMatching){
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


		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		Matrix newPosition = gn->getRawEncoderPosition() + Wk * zl;
		
		gn->setPosition(newPosition(0, 0), newPosition(1, 0), newPosition(2, 0));
		float angle = 0;
		bool isInsidePolygon = gn->getCurrentSector()->checkPointXYInPolygon(PointXY(newPosition(0, 0), newPosition(1, 0)), angle);

		if(not isInsidePolygon){
			gn->loadSector(gn->getCurrenMapId(), gn->getNextSectorId());
			RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", gn->getNextSectorId(), gn->getCurrentSector()->getName().c_str());
			gn->setNextSectorId(RN_NONE);
			//new position
			gn->setLastVisitedNode(0);
	        gn->setPosition(newPosition(0, 0) + gn->getNextSectorCoord().getX(), newPosition(1, 0) + gn->getNextSectorCoord().getY(), newPosition(2, 0));
			gn->initializeKalmanVariables();
		}
	} else {
		init();
	}

	RNUtils::sleep(10);
}

void RNKalmanLocalizationTask::getObservations(Matrix& observations){
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
	int laserIndex = 0, cameraIndex = laserLandmarksCount, rfidIndex = (laserLandmarksCount + cameraLandmarksCount);
	Matrix result(totalLandmarks, 3);

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
		result(zIndex, 2) = (float)landmark->id;
	}

	observations = result;
}

void RNKalmanLocalizationTask::landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle){
	distance = std::sqrt(std::pow(landmark->xpos - Xk(0, 0), 2) + std::pow(landmark->ypos - Xk(1, 0), 2));
	angle = std::atan2(landmark->ypos - Xk(1, 0), landmark->xpos - Xk(0, 0)) - Xk(2, 0);
}