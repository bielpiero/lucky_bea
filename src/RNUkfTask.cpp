#include "RNUkfTask.h"

const double RNUkfTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNUkfTask::CAMERA_ERROR_POSITION_Y = -0.014;

RNUkfTask::RNUkfTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	currentSector = NULL;
	test = std::fopen("laser_camera.txt","w+");
	this->startThread();
}

RNUkfTask::~RNUkfTask(){
	if(test != NULL){
		std::fclose(test);
	}
}

void RNUkfTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
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


void RNUkfTask::task(){
	if(enableLocalization){
		int rsize = 0, vsize = 0;
		int activeRL = 0, activeVL = 0;
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
		
		Matrix currentQ = gn->getQ();
		if(deltaDistance == 0.0 and deltaAngle == 0.0){
			currentQ = Matrix(2, 2);
		}
		RNUtils::getOdometryPose(xk, deltaDistance, deltaAngle, xk_1);

		Pk = (Ak * pk1 * ~Ak) + (Bk * currentQ * ~Bk);
		if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
			printf("Error gordo...\n");
		}
		Matrix zkl;
		getObservations(zkl);

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
		Matrix zl(sizeH, 1);
		if(sizeH > 1){
			Hk = Matrix(sizeH, STATE_VARIABLES);
			int laserIndex = 0, cameraIndex = 0;
			if(gn->isLaserSensorActivated()){
				cameraIndex = 2 * activeRL;
				rsize = gn->getLaserLandmarks()->size();
				for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
					RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

					//RNUtils::printLn("Laser landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
					std::map<int, double> euclideanDistances;
					std::map<int, Matrix> matricesHk;
					Matrix smallR(2, 2);
					smallR(0, 0) = gn->getLaserDistanceVariance();
					smallR(1, 1) = gn->getLaserAngleVariance();
					currentR(2 * i, 2 * i) = smallR(0, 0);
					currentR(2 * i + 1, 2 * i + 1) = smallR(1, 1);

					Matrix measure(2, 1);
					measure(0, 0) = lndmrk->getPointsXMean();
					measure(1, 0) = lndmrk->getPointsYMean();
					
					for (int j = 0; j < laserLandmarksCount; j++){
						s_landmark* currLandmark = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_LASER_STR, (int)zkl(j, 3));
						//printf("L[%d]: x: %f, y %f\n", currLandmark->id, currLandmark->xpos, currLandmark->ypos);
						Matrix smallHk(2, 3);
						if(currLandmark != NULL and currentSector->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){
							double landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, xk_1(0, 0), xk_1(1, 0));
							smallHk(0, 0) = -(currLandmark->xpos - xk_1(0, 0))/landmarkDistance;
							smallHk(0, 1) = -(currLandmark->ypos - xk_1(1, 0))/landmarkDistance;
							smallHk(0, 2) = 0.0;

							smallHk(1, 0) = (currLandmark->ypos - xk_1(1, 0))/std::pow(landmarkDistance, 2);
							smallHk(1, 1) = -(currLandmark->xpos - xk_1(0, 0))/std::pow(landmarkDistance, 2);
							smallHk(1, 2) = -1.0;
						}
						
						matricesHk.emplace(j, smallHk);
						Matrix omega = smallHk * Pk * ~smallHk + smallR;
						Matrix obs(2, 1);
						obs(0, 0) = zkl(j, 0);
						obs(1, 0) = zkl(j, 1);


						Matrix mdk = ~(measure - obs) * omega * (measure - obs);

						euclideanDistances.emplace(j, std::sqrt(std::abs(mdk(0, 0))));
					}
					auto minorDistance = std::min_element(euclideanDistances.begin(), euclideanDistances.end(), [](std::pair<int, double> x, std::pair<int, double> y){ return x.second < y.second; });
					if(minorDistance != euclideanDistances.end()){
						Matrix smallHk = matricesHk[minorDistance->first];
						Hk(2 * i, 0) = smallHk(0, 0);
						Hk(2 * i, 1) = smallHk(0, 1);
						Hk(2 * i, 2) = smallHk(0, 2);

						Hk(2 * i + 1, 0) = smallHk(1, 0);
						Hk(2 * i + 1, 1) = smallHk(1, 1);
						Hk(2 * i + 1, 2) = smallHk(1, 2);

						zl(2 * i, 0) = lndmrk->getPointsXMean() - zkl(minorDistance->first, 0);
						zl(2 * i + 1, 0) = lndmrk->getPointsYMean() - zkl(minorDistance->first, 1);
						if(zl(2 * i + 1, 0) > M_PI){
							zl(2 * i + 1, 0) = zl(2 * i + 1, 0) - 2 * M_PI;
						} else if(zl(2 * i + 1, 0) < -M_PI){
							zl(2 * i + 1, 0) = zl(2 * i + 1, 0) + 2 * M_PI;
						}

						double mdDistance = std::abs(zl(2 * i, 0) / std::sqrt(gn->getLaserDistanceVariance()));
						double mdAngle = std::abs(zl(2 * i + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));

						if (mdDistance > laserTMDistance or mdAngle > laserTMAngle){
		
							//RNUtils::printLn("landmark %d rejected...", indexFound);
							zl(2 * i, 0) = 0.0;
							zl(2 * i + 1, 0) = 0.0;
							rsize--;
						}
					}
					
					
				}
			}

			if(gn->isCameraSensorActivated()){
				vsize = gn->getVisualLandmarks()->size();
				for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
					RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
					currentR(cameraIndex + i, cameraIndex + i) = gn->getCameraAngleVariance();

					int zklIndex = RN_NONE;
					for(int j = laserLandmarksCount; j < laserLandmarksCount + cameraLandmarksCount and (zklIndex == RN_NONE); j++){
						s_landmark* currLandmark = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_CAMERA_STR, (int)zkl(j, 3));
						if(currLandmark != NULL and currLandmark->type == XML_SENSOR_TYPE_CAMERA_STR){
							if(lndmrk != NULL and currLandmark->id == lndmrk->getMarkerId()){
								double nrx, nry;
								Matrix disp = Matrix(2, 1);
								RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk_1(2, 0), &nrx, &nry);
								disp(0, 0) = nrx;
								disp(1, 0) = nry;

								double landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (xk_1(0, 0) + disp(0, 0)), (xk_1(1, 0) + disp(1, 0)));

								Hk(cameraIndex + i, 0) = (currLandmark->ypos - (xk_1(1, 0) + disp(1, 0)))/std::pow(landmarkDistance, 2);
								Hk(cameraIndex + i, 1) = -(currLandmark->xpos - (xk_1(0, 0) + disp(0, 0)))/std::pow(landmarkDistance, 2);
								Hk(cameraIndex + i, 2) = -1.0;

								zklIndex = j;
							}
						}
					}

					if(lndmrk != NULL and (lndmrk->getMapId() == currentSector->getMapId()) and (lndmrk->getSectorId() == currentSector->getId()) and zklIndex != RN_NONE){

						double angleFixed = lndmrk->getPointsYMean();
						

						zl(cameraIndex + i, 0) = angleFixed - zkl(zklIndex, 1);
						if(zl(cameraIndex + i, 0) > M_PI){
							zl(cameraIndex + i, 0) = zl(cameraIndex + i, 0) - 2 * M_PI;
						} else if(zl(cameraIndex + i, 0) < -M_PI){
							zl(cameraIndex + i, 0) = zl(cameraIndex + i, 0) + 2 * M_PI;
						}
						//RNUtils::printLn("MapId: %d, SectorId: %d, markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {a: %f}", lndmrk->getMapId(), lndmrk->getSectorId(), lndmrk->getMarkerId(), distanceFixed, angleFixed, zkl(j, 0), zkl(j, 1), zl(j + laserOffset, 0));
					}

					double mdAngle = std::abs(zl(cameraIndex + i, 0) / std::sqrt(gn->getCameraAngleVariance()));

					if (mdAngle > cameraTMAngle){
						//RNUtils::printLn("Landmark %d rejected...", (int)zkl(i, 3));
						zl(cameraIndex + i, 0) = 0;
						vsize--;
					}
				}
			}

			Matrix Sk = Hk * Pk * ~Hk + currentR;
			Matrix Wk = Pk * ~Hk * !Sk;

			//printf("Zl:\n");
			//zl.print();

			char bufferpk1[256], bufferpk[256];
			sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
			if(Pk(0, 0) < 0.0 or Pk(1, 1) < 0 or Pk(2, 2) < 0){
				printf("Error gordo...\n");
			}
			sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			xk = xk_1 + Wk * zl;
			xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));
			gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
			char buffer[1024];
			sprintf(buffer, "%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%s\t%s\t%d\t%d\n", gn->getRawEncoderPosition()(0, 0), gn->getRawEncoderPosition()(1, 0), gn->getRawEncoderPosition()(2, 0), xk(0, 0), xk(1, 0), xk(2, 0), bufferpk1, bufferpk, rsize, vsize);
			if(test != NULL){
				fprintf(test, "%s", buffer);
			}
		} else {
			char bufferpk1[256], bufferpk[256];
			sprintf(bufferpk1, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			Pk = Pk;
			sprintf(bufferpk, "%.4e\t%.4e\t%.4e", Pk(0, 0), Pk(1, 1), Pk(2, 2));
			xk = xk_1;
			xk(2, 0) = RNUtils::fixAngleRad(xk(2, 0));
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


void RNUkfTask::getObservations(Matrix& observations){
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

	for(int k = 0; k < currentSector->landmarksSize(); k++){
		int zIndex = RN_NONE;
		double distance = 0, angle = 0;
		Matrix disp = Matrix(2, 1);
		s_landmark* landmark = currentSector->landmarkAt(k);
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