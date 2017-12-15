#include "RNFactorySensorsTask.h"

RNFactorySensorsTask::RNFactorySensorsTask(const RobotNode* rn, const char* name, const char* description) : RNRecurrentTask(rn, name, description){
	prevRawPose = ArPose(0.0, 0.0, 0.0);
    this->isFirstFakeEstimation = true;
}

RNFactorySensorsTask::~RNFactorySensorsTask(){

}

void RNFactorySensorsTask::task(){
	getRawRobotPosition();
    rn->getBatterChargeStatus();
    getBumpersStatus();
    rn->getRobotPosition();
    getSonarsScan();
    rn->onSensorsScanCompleted();
}

void RNFactorySensorsTask::getRawRobotPosition(){
    //computePositionFromEncoders();
    ArPose *myRawPose = new ArPose(rn->getRobot()->getPose());
    rn->onRawPositionUpdate(myRawPose->getX()/1.0e3, myRawPose->getY()/1.0e3, myRawPose->getThRad(), this->deltaDistance/1.0e3, this->deltaDegrees);
    delete myRawPose;
    myRawPose = NULL;
}

void RNFactorySensorsTask::computePositionFromEncoders(void){
    ArPose *currentRawPose = new ArPose(rn->getRobot()->getPose());

    double currentDistance = rn->getRobot()->getOdometerDistance();
    //double currentDistance = RNUtils::distanceTo(currentRawPose->getX(), currentRawPose->getY(), prevRawPose.getX(), prevRawPose.getY());
    double currentRads = RNUtils::deg2Rad(rn->getRobot()->getOdometerDegrees());

    double currentVel = rn->getRobot()->getVel();
    double currentRotVel = rn->getRobot()->getRotVel();
    
    //RNUtils::printLn("odometer: {d: %lf, th: %lf}, vel: {lin: %lf, rot: %lf}", currentDistance, currentRads, robot->getVel(), robot->getRotVel());

    if(isFirstFakeEstimation){
        prevDistance = currentDistance;
        prevRads = currentRads;
        prevVel = currentVel;
        prevRotVel = currentRotVel;
        isFirstFakeEstimation = false;
    }

    deltaDistance = currentDistance - prevDistance;
    deltaDegrees = currentRads - prevRads;

    if(rn->getRobot()->getVel() < 0){
        deltaDistance *= -1.0;
    }

    if(currentRotVel < 0){
        if(deltaDegrees > 0){
            deltaDegrees *= -1.0;    
        }
    }
    rn->setIncrementPosition(deltaDistance/1.0e3, deltaDegrees);
    
    /*if(deltaDistance != 0.0 and deltaDegrees != 0.0){
        RNUtils::printLn("delta: {%f, %f}", deltaDistance/1.0e3, deltaDegrees);
        getRawPoseFromOdometry(currentRawPose);
    }*/

    prevRawPose.setX(currentRawPose->getX());
    prevRawPose.setY(currentRawPose->getY());
    prevRawPose.setTh(currentRawPose->getTh());

    prevVel = currentVel;
    prevRotVel = currentRotVel;
    prevDistance = currentDistance;
    prevRads = currentRads;

    delete currentRawPose;
    currentRawPose = NULL;
    RNUtils::sleep(20);
    //rn->getRobot()->resetTripOdometer();
    
}

void RNFactorySensorsTask::getRawPoseFromOdometry(ArPose* rawPose){
    double x = 0, y = 0, th = 0;
    
    x = rawPose->getX() + deltaDistance * std::cos(rawPose->getThRad() + (deltaDegrees / 2.0));
    y = rawPose->getY() + deltaDistance * std::sin(rawPose->getThRad() + (deltaDegrees / 2.0));
    th = rawPose->getThRad() + deltaDegrees;
    
    RNUtils::printLn("Raw{x: %.2lf, y: %.2lf, \u03d1: %.2lf}, P{x: %.2lf, y: %.2lf, \u03d1: %.2lf}", rawPose->getX(), rawPose->getY(), rawPose->getThRad(), x, y, th);

    //rn->setPosition(x / 1000.0, y / 1000.0, th);

}

void RNFactorySensorsTask::getBumpersStatus(void){
    std::vector<bool> frontBumpers(rn->getRobot()->getNumFrontBumpers());
    std::vector<bool> rearBumpers(rn->getRobot()->getNumRearBumpers());

    int stall = rn->getRobot()->getStallValue();
    unsigned char front_bumpers = (unsigned char)(stall >> 8);
    unsigned char rear_bumpers = (unsigned char)(stall);

    for (unsigned int i = 0; i < rn->getRobot()->getNumFrontBumpers(); i++){
        frontBumpers.at(i) = (front_bumpers & (1 << (i + 1))) == 0 ? false : true;
    }

    for (unsigned int i = 0; i < rn->getRobot()->getNumRearBumpers(); i++){
        rearBumpers.at(i) = (rear_bumpers & (1 << (rn->getRobot()->getNumRearBumpers() - i))) == 0 ? false : true;
    }

    rn->onBumpersUpdate(frontBumpers, rearBumpers);
}

void RNFactorySensorsTask::getSonarsScan(void){
    if(rn->getSonarsStatus()){
        std::vector<PointXY*>* data = new std::vector<PointXY*>();
        for (int i = 0; i < rn->getRobot()->getNumSonar(); i++) {
            ArSensorReading* reading = NULL;
            reading = rn->getRobot()->getSonarReading(i);
            if(reading) {
                data->push_back(new PointXY(reading->getLocalX() / 1000.0, reading->getLocalY() / 1000.0));
            }
            delete reading;
        }
        rn->onSonarsDataUpdate(data);
        for (int i = 0; i < data->size(); i++) {
            delete data->at(i);
        }
        data->clear();
        delete [] data;
    }
}

void RNFactorySensorsTask::onKilled(){
	this->isFirstFakeEstimation = true;
    prevRawPose.setX(0.0);
    prevRawPose.setY(0.0);
    prevRawPose.setTh(0.0);
}