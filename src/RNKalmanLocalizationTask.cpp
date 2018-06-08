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
		rfidLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);

		laserTMDistance = gn->getLaserDistanceAlpha() / std::sqrt(gn->getLaserDistanceVariance());
		laserTMAngle = gn->getLaserAngleAlpha() / std::sqrt(gn->getLaserAngleVariance());
		//cameraTMDistance = gn->getCameraDistanceAlpha() / std::sqrt(gn->getCameraDistanceVariance());
		cameraTMDistance = 0.1;
		cameraTMAngle = gn->getCameraAngleAlpha() / std::sqrt(gn->getCameraAngleVariance());
		rfidTMDistance = 0.2;
		rfidTMAngle = 0.2;

		xk = gn->getRawEncoderPosition();
		xk.print();
		gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		//gn->unlockLaserLandmarks();
		//gn->unlockVisualLandmarks();
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
		/*char buff[1024];
		sprintf(buff, "%f\t%f\t%f\t%f\t%f\n", gn->getRobot()->getX(), gn->getRobot()->getY(), gn->getRobot()->getTh() * M_PI / 180.0, deltaDistance, deltaAngle);
		if(test != NULL){
			fprintf(test, "%s", buff);
		}*/
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

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (xk_1(0, 0) + disp(0, 0)), (xk_1(1, 0) + disp(1, 0)));

					Hk(zIndex, 0) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex, 1) = -(currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex, 2) = 0.0;

					Hk(zIndex + 1, 0) = (currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex + 1, 1) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex + 1, 2) = -1.0;
				}
			}
			if(gn->isCameraSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
					zIndex = cameraIndex;
					cameraIndex++;

					disp(0, 0) = CAMERA_ERROR_POSITION_X * std::cos(xk_1(2, 0)) - CAMERA_ERROR_POSITION_Y * std::sin(xk_1(2, 0));
					disp(1, 0) = CAMERA_ERROR_POSITION_X * std::sin(xk_1(2, 0)) + CAMERA_ERROR_POSITION_Y * std::cos(xk_1(2, 0));

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (xk_1(0, 0) + disp(0, 0)), (xk_1(1, 0) + disp(1, 0)));

					Hk(zIndex, 0) = (currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex, 1) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex, 2) = -1.0;
				}
			}
			disp(0, 0) = 0.0;
			disp(1, 0) = 0.0;
		}
		//printf("Hk\n");
		//Hk.print();
		/*Hk = Matrix(2 * totalLandmarks, STATE_VARIABLES);
		int laserIndex = 0, cameraIndex = 0, rfidIndex = 0;
		if(gn->isLaserSensorActivated()){
			cameraIndex += laserLandmarksCount;
			rfidIndex += laserLandmarksCount;
		}
		if(gn->isCameraSensorActivated()){
			rfidIndex += cameraLandmarksCount;
		}
		for(int i = 0, zIndex = 0; i < gn->getCurrentSector()->landmarksSize(); i++){
			Matrix disp = Matrix(2, 1);
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

					disp(0, 0) = CAMERA_ERROR_POSITION_X;
					disp(1, 0) = CAMERA_ERROR_POSITION_Y;
				}
			}
			if(gn->isRfidSensorActivated()){
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_RFID_STR){
					zIndex = 2 * rfidIndex;
					rfidIndex++;
				}
			}
			s_landmark* currLandmark = gn->getCurrentSector()->landmarkAt(i);
			double landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (xk_1(0, 0) + disp(0, 0)), (xk_1(1, 0) + disp(1, 0)));

			Hk(zIndex, 0) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/landmarkDistance;
			Hk(zIndex, 1) = -(currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/landmarkDistance;
			Hk(zIndex, 2) = 0.0;

			Hk(zIndex + 1, 0) = (currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/landmarkDistance;
			Hk(zIndex + 1, 1) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/landmarkDistance;
			Hk(zIndex + 1, 2) = -1.0;
		}*/

		Matrix zkl;
		getObservations(zkl);
		//printf("Zkl\n");
		//zkl.print();
		//Matrix zl(2 * totalLandmarks, 1);
		Matrix zl(sizeH, 1);

		laserIndex = 0;
		cameraIndex = 0; 

		if(gn->isLaserSensorActivated()){
			cameraIndex = laserLandmarksCount;
		}

		/*if(gn->isLaserSensorActivated()){
			cameraIndex += laserLandmarksCount;
			rfidIndex += laserLandmarksCount;
		}
		if(gn->isCameraSensorActivated()){
			rfidIndex += cameraLandmarksCount;
		}*/
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
					smallHk(2, 1) = Hk(2 * j + 1, 1);
					smallHk(3, 2) = Hk(2 * j + 1, 2);
					Matrix omega = smallHk * Pk * ~smallHk + smallR;
					//float ed = std::sqrt(std::pow(zkl(j, 0), 2.0) + std::pow(lndmrk->getPointsXMean(), 2.0) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
					Matrix obs(2, 1);
					obs(0, 0) = zkl(j, 0);
					obs(1, 0) = zkl(j, 1);
					Matrix mdk = ~(measure - obs) * omega * (measure - obs);
					float md = std::sqrt(mdk(0, 0));
					euclideanDistances.push_back(std::pair<int, float>(j, md));
					
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
					//RNUtils::printLn("markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {d: %f, a: %f}", indexFound, lndmrk->getPointsXMean() , lndmrk->getPointsYMean() , zkl(indexFound , 0), zkl(indexFound , 1), zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));
					
					double mdDistance = std::abs(zl(2 * indexFound, 0) / std::sqrt(gn->getLaserDistanceVariance()));
					double mdAngle = std::abs(zl(2 * indexFound + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));
					//printf("Mhd: (%d), %lg, %lg, nud: %lg, nua: %lg\n", i, mdDistance, mdAngle, zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));
					if (mdDistance > laserTMDistance or mdAngle > laserTMAngle){
						//distanceThreshold = (máxima zl que permite un buen matching)/sigma
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
							
							//zl(2 * j, 0) = distanceFixed - zkl(j, 0);
							//zl(2 * j, 0) = 0.0;
							/*if(distanceFixed > 1.0){
								currentR(2 * j, 2 * j) = currentR(2 * j, 2 * j) * std::exp(std::abs(distanceFixed - zkl(j, 0)));
							}*/
							/*zl(2 * j + 1, 0) = angleFixed - zkl(j, 1);
							if(zl(2 * j + 1, 0) > M_PI){
								zl(2 * j + 1, 0) = zl(2 * j + 1, 0) - 2 * M_PI;
							} else if(zl(2 * j + 1, 0) < -M_PI){
								zl(2 * j + 1, 0) = zl(2 * j + 1, 0) + 2 * M_PI;
							}*/
							zl(j + laserOffset, 0) = angleFixed - zkl(j, 1);
							if(zl(j + laserOffset, 0) > M_PI){
								zl(j + laserOffset, 0) = zl(j + laserOffset, 0) - 2 * M_PI;
							} else if(zl(j + laserOffset, 0) < -M_PI){
								zl(j + laserOffset, 0) = zl(j + laserOffset, 0) + 2 * M_PI;
							}
							//RNUtils::printLn("MapId: %d, SectorId: %d, markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {a: %f}", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(j + laserOffset, 0));
						}
					}
				} else { 
					std::vector<std::pair<int, double> > cameraDistances;
					//for (int j = cameraIndex; j < (cameraIndex + cameraLandmarksCount); j++){
					for (int j = cameraIndex; j < zkl.rows_size(); j++){
						//double cd = std::sqrt(std::pow(zkl(j, 0), 2.0) + std::pow(lndmrk->getPointsXMean(), 2.0) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1))));
						double cd = std::abs(lndmrk->getPointsYMean() - zkl(j, 1));
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

						//zl(2 * indexFound, 0) = distanceFixed - zkl(indexFound, 0);

						/*zl(2 * indexFound + 1, 0) = angleFixed - zkl(indexFound, 1);
						if(zl(2 * indexFound + 1, 0) > M_PI){
							zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) - 2 * M_PI;
						} else if(zl(2 * indexFound + 1, 0) < -M_PI){
							zl(2 * indexFound + 1, 0) = zl(2 * indexFound + 1, 0) + 2 * M_PI;
						}*/

						zl(indexFound + laserOffset, 0) = angleFixed - zkl(indexFound + laserOffset, 1);
						if(zl(indexFound + laserOffset, 0) > M_PI){
							zl(indexFound + laserOffset, 0) = zl(indexFound + laserOffset, 0) - 2 * M_PI;
						} else if(zl(indexFound + laserOffset, 0) < -M_PI){
							zl(indexFound + laserOffset, 0) = zl(indexFound + laserOffset, 0) + 2 * M_PI;
						}
					}

				}
			}
			//Calculates Mahalanobis distance to determine whether a measurement is acceptable.				
			//for (int i = cameraIndex; i < (cameraIndex + cameraLandmarksCount); i++){
			for (int i = cameraIndex; i < zkl.rows_size(); i++){
				//double mdDistance = std::abs(zl(2 * i, 0) / std::sqrt(gn->getCameraDistanceVariance()));
				//double mdDistance = 0;
				double mdAngle = std::abs(zl(i + laserOffset, 0) / std::sqrt(gn->getCameraAngleVariance()));
				//printf("Mh: (%d): %g. nu: %g\n", i, mdAngle, zl(i + laserOffset, 0));
				if (/*mdDistance > cameraTMDistance or */mdAngle > cameraTMAngle){
					//RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
					//distanceThreshold = (máxima zl que permite un buen matching)/sigma
					//zl(2 * i, 0) = 0;
					zl(i + laserOffset, 0) = 0;
					vsize--;
				}
			}	
			gn->getVisualLandmarks()->clear();
			gn->unlockVisualLandmarks();
			//RNUtils::printLn("-----------> Zl, Variance: %g", gn->getCameraAngleVariance());
			//zl.print();
		}
		//RNUtils::printLn("zl");
		//zl.print();

		Matrix Sk = Hk * Pk * ~Hk + currentR;
		Matrix Wk = Pk * ~Hk * !Sk;

		//printf("zl\n");
		//zl.print();

		Wk = fixFilterGain(Wk, zl);

		//RNUtils::printLn("Wk");
		//Wk.print();

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
		bool isInsidePolygon = gn->getCurrentSector()->checkPointXYInPolygon(PointXY(xk(0, 0), xk(1, 0)));
		if(not isInsidePolygon){
			//gn->loadSector(gn->getCurrenMapId(), gn->getNextSectorId());
			//RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", gn->getNextSectorId(), gn->getCurrentSector()->getName().c_str());
			//gn->setNextSectorId(RN_NONE);
			//new position
			//gn->setLastVisitedNode(0);
	        //gn->setPosition(newPosition(0, 0) + gn->getNextSectorCoord().getX(), newPosition(1, 0) + gn->getNextSectorCoord().getY(), newPosition(2, 0));
	        //init();
		}
	} else {
		init();
	}
}

Matrix RNKalmanLocalizationTask::fixFilterGain(const Matrix wk, const Matrix z){
	Matrix result = wk;
	for(int i = 0; i < z.rows_size(); i++){
		if(z(i, 0) == 0.0){
			for(int j = 0; j < wk.rows_size(); j++){
				result(j, i) = 0.0;
			}
		}
	}
	return result;
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

	for(int k = 0; k < gn->getCurrentSector()->landmarksSize(); k++){
		int zIndex = RN_NONE;
		double distance = 0, angle = 0;
		Matrix disp = Matrix(2, 1);
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);
		if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
			disp(0, 0) = CAMERA_ERROR_POSITION_X * std::cos(xk_1(2, 0)) - CAMERA_ERROR_POSITION_Y * std::sin(xk_1(2, 0));
			disp(1, 0) = CAMERA_ERROR_POSITION_X * std::sin(xk_1(2, 0)) + CAMERA_ERROR_POSITION_Y * std::cos(xk_1(2, 0));
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