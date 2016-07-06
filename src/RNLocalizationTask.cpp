#include "RNLocalizationTask.h"

RNLocalizationTask::RNLocalizationTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	enableLocalization = false;
}

RNLocalizationTask::~RNLocalizationTask(){

}

void RNLocalizationTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
		Pk = gn->getP();
		alpha = 0.2;
		enableLocalization = true;
	} else{
		enableLocalization = false;
	}
}

void RNLocalizationTask::onKilled(){
	enableLocalization = false;
}

void RNLocalizationTask::task(){
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

		Hk = Matrix(2 * gn->getCurrentSector()->landmarksSize(), STATE_VARIABLES);
		for(int i = 0, zIndex = 0; i < gn->getCurrentSector()->landmarksSize(); i++, zIndex += 2){
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
		gn->lockLaserLandmarks();

		Matrix zl(2 * gn->getCurrentSector()->landmarksSize(), 1);
		for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
			RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

			//RNUtils::printLn("landmarks {d: %f, a: %f}\n", lndmrk->getPointsXMean(), lndmrk->getPointsYMean());
			std::vector<float> euclideanDistances;
			for (int j = 0; j < zkl.rows_size(); j++){
				euclideanDistances.push_back(std::sqrt(std::pow(zkl(j, 0), 2) + std::pow(lndmrk->getPointsXMean(), 2) - (2 * zkl(j, 0) * lndmrk->getPointsXMean() * std::cos(lndmrk->getPointsYMean() - zkl(j, 1)))));
				//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
			}

			float minorDistance = std::numeric_limits<float>::infinity();
			int indexFound = RN_NONE;
			for (int j = 0; j < euclideanDistances.size(); j++){
				if(euclideanDistances.at(j) < alpha){
					minorDistance = euclideanDistances.at(j);
					indexFound = j;
				}	
			}
			
			//RNUtils::printLn("Matched landmark: {idx : %d, MHD: %f}\n", indexFound, minorDistance);
			if(indexFound > RN_NONE){
				zl(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl(indexFound, 0);
				zl(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl(indexFound, 1);
			}
		}
		gn->unlockLaserLandmarks();

		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		Matrix newPosition = gn->getRawEncoderPosition() + Wk * zl;
		//newPosition.print();

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
	//RNUtils::sleep(20);
}

void RNLocalizationTask::getObservations(Matrix& observations){
	Matrix result(gn->getCurrentSector()->landmarksSize(), 2);

	for(int k = 0; k < gn->getCurrentSector()->landmarksSize(); k++){
		float distance = 0, angle = 0;
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);


		landmarkObservation(gn->getRawEncoderPosition(), landmark, distance, angle);
		result(k, 0) = distance;
		if(angle > M_PI){
			angle = angle - 2 * M_PI;
		} else if(angle < -M_PI){
			angle = angle + 2 * M_PI;
		}
		result(k, 1) = angle;
	}

	observations = result;
}

void RNLocalizationTask::landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle){
	distance = std::sqrt(std::pow(landmark->xpos - Xk(0, 0), 2) + std::pow(landmark->ypos - Xk(1, 0), 2));
	angle = std::atan2(landmark->ypos - Xk(1, 0), landmark->xpos - Xk(0, 0)) - Xk(2, 0);
}