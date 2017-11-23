#include "RNLaserTask.h"

const float RNLaserTask::SECURITY_DISTANCE = 0.5;
const float RNLaserTask::LASER_MAX_RANGE = 11.6;
const float RNLaserTask::LANDMARK_RADIUS = 0.045;

RNLaserTask::RNLaserTask(GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = gn;
	laserConnector = new ArLaserConnector(rn->getArgumentParser(), rn->getRobot(), rn->getRobotConnector());
	if(!laserConnector->connectLasers(false, true, false)){
		printf("Could not connect to configured lasers.\n");
	}

	laser = rn->getRobot()->findLaser(1);
	if(!laser){
		printf("Error. Not Connected to any laser.\n");
	} else {
		printf("Connected to SICK LMS200 laser.\n");
	}
	laserDataScan = NULL;
	laserLandmarks = NULL;

	//pthread_mutex_init(&mutexSensorsReadingsLocker, NULL);
}

RNLaserTask::~RNLaserTask(){
	//delete laser;
}


void RNLaserTask::onKilled(){
	
}

void RNLaserTask::getLaserScan(void){
	if(laser != NULL){
		laser->lockDevice();
        
		const std::list<ArSensorReading*> *currentReadings = new std::list<ArSensorReading*>(*laser->getRawReadings());
        //lockSensorsReadings();
        if(laserDataScan == NULL){
            laserDataScan = new LaserScan();
        } else {
		  laserDataScan->clear();
        }
		for(std::list<ArSensorReading*>::const_iterator it = currentReadings->begin(); it != currentReadings->end(); ++it){
			laserDataScan->addLaserScanData((float)(*it)->getRange() / 1000, (float)(*it)->getExtraInt());
		}
        
        laser->unlockDevice();

        delete currentReadings;
        //unlockSensorsReadings();
	}
}

void RNLaserTask::securityDistanceChecker(){
    
    if(laserDataScan != NULL){
        
        bool thereIsSomethingInFront = false;
        for(int it = MIN_INDEX_LASER_SECURITY_DISTANCE; (it < (laserDataScan->size() - MAX_INDEX_LASER_SECURITY_DISTANCE)) and not thereIsSomethingInFront; it++){
            if(laserDataScan->getRange(it) < SECURITY_DISTANCE){
                thereIsSomethingInFront = true;           
            }
        }
        if(thereIsSomethingInFront){
            if(gn->isDirectMotion() and gn->isGoingForward() and not gn->isNotAllowedToMove()){
                gn->notAllowedToMove(true);
                gn->getRobot()->stop();
            } else if(gn->isGoalActive()){
                gn->deactivateGoal();
            }
        } else {
        	gn->notAllowedToMove(false);
            if(not gn->isGoalActive()){
                gn->activateGoal();
            }
        }
    }
}

void RNLaserTask::getReflectiveLandmarks(){
	
	float angle_min = laserDataScan->getAngleMin();
	float angle_max = laserDataScan->getAngleMax();
	float angle_increment = laserDataScan->getIncrement();
	std::vector<float>* data = laserDataScan->getRanges();
	std::vector<float> dataIntensities = *laserDataScan->getIntensities();

	std::vector<int> dataIndices = stats::findIndicesHigherThan(dataIntensities, 0);

	gn->lockLaserLandmarks();
	laserLandmarks = gn->getLaserLandmarks();

	laserLandmarks->clear();
	RNLandmark* current = new RNLandmark;


	for(int i = 0; i < dataIndices.size(); i++){
		if(i < dataIndices.size() - 1){
			if((dataIndices[i + 1] - dataIndices[i]) <= 10){
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				
			} else {
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
				current = new RNLandmark;
			}
		} else {
			if((dataIndices[i] - dataIndices[i - 1]) <= 10){
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
			} else if(current->size() > 0){
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
			}
		}
	}
	
	std::ostringstream buffFile;
	for(int i = 0; i < laserLandmarks->size(); i++){
		
		//RNUtils::printLn("Questa Merda prima alla correzione [%d] è {d: %f, a: %f}", i, laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS, laserLandmarks->at(i)->getPointsYMean());
		Matrix Pkl = Matrix::eye(2);
		Matrix Rkl = std::pow(0.003, 2) * Matrix::eye(laserLandmarks->at(i)->size());
		Matrix Zk(laserLandmarks->at(i)->size(), 1);
		Matrix Zke(laserLandmarks->at(i)->size(), 1);
		Matrix Hkl(laserLandmarks->at(i)->size(), 2);
		Matrix Pc(2, 1);
		Pc(0, 0) = laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS;
		Pc(1, 0) = laserLandmarks->at(i)->getPointsYMean();

		//fprintf(file, "$MEAN_LASER_RAW\t%d\t%f\t%f\n", (i + 1), Pc(0, 0), Pc(1, 0));
		for(int j = 0; j < laserLandmarks->at(i)->size(); j++){
			Zk(j, 0) = laserLandmarks->at(i)->getPointAt(j)->getX();
			Zke(j, 0) = Pc(0, 0) * cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) - LANDMARK_RADIUS * cos(asin((Pc(0, 0) / LANDMARK_RADIUS) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY())));

			//fprintf(file, "$LASER_POINT\t%d\t%d\t%f\t%f\t%f\n", (i + 1), (j + 1), Zk(j, 0), Zke(j, 0), laserLandmarks->at(i)->getPointAt(j)->getY());

			float calc = (1 / LANDMARK_RADIUS) * (1 / std::sqrt(1 - ((std::pow(Pc(0, 0), 2) * std::pow(sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()), 2))/(std::pow(LANDMARK_RADIUS, 2)))));
			Hkl(j, 0) = cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) + Pc(0, 0) * std::pow(sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()), 2) * calc;
			Hkl(j, 1) = -Pc(0, 0) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) + std::pow(Pc(0, 0), 2) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) * cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) * calc;
		}

		Matrix Skl = Hkl * Pkl * ~Hkl + Rkl;
		Matrix Wkl = Pkl * ~Hkl * !Skl;
		Pc = Pc + (Wkl * (Zk - Zke));
		//fprintf(file, "$MEAN_LASER_FIXED\t%d\t%f\t%f\n", (i + 1), Pc(0, 0), Pc(1, 0));
		laserLandmarks->at(i)->setPointsXMean(Pc(0, 0));
		laserLandmarks->at(i)->setPointsYMean(Pc(1, 0));

		//RNUtils::printLn("Questa Merda dopo della correzione [%d] è {d: %f, a: %f}", i, laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS, laserLandmarks->at(i)->getPointsYMean());
		
	}
	gn->unlockLaserLandmarks();
	gn->setLaserLandmarks(laserLandmarks);
	
}

void RNLaserTask::task(){
	getLaserScan();
	securityDistanceChecker();
	getReflectiveLandmarks();
	RNUtils::sleep(10);
}