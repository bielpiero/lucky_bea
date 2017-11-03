#include "RNLaserTask.h"

RNLaserTask::RNLaserTask(GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	
	laserConnector = new ArLaserConnector(rn->getArgumentParser(), rn->getRobot(), rn->getRobotConnector());
	if(!laserConnector->connectLasers(false, false, true)){
		printf("Could not connect to configured lasers.\n");
	}

	laser = rn->getRobot()->findLaser(1);
	if(!laser){
		printf("Error. Not Connected to any laser.\n");
	} else {
		printf("Connected to SICK LMS200 laser.\n");
	}
}

RNLaserTask::~RNLaserTask(){
	//delete laser;
}

void RNLaserTask::init(){
	
}

void RNLaserTask::onKilled(){
	
}

void RNLaserTask::task(){
	
	RNUtils::sleep(10);
}