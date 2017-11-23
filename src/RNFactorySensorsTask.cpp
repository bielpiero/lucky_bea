#include "RNFactorySensorsTask.h"

RNFactorySensorsTask::RNFactorySensorsTask(const RobotNode* rn, const char* name, const char* description) : RNRecurrentTask(rn, name, description){
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
    computePositionFromEncoders();
    ArPose *myRawPose = new ArPose(rn->getRobot()->getPose());
    rn->onRawPositionUpdate(myRawPose->getX()/1e3, myRawPose->getY()/1e3, myRawPose->getThRad(), this->deltaDistance, this->deltaDegrees);
    delete myRawPose;
    myRawPose = NULL;
}

void RNFactorySensorsTask::computePositionFromEncoders(void){
    double currentDistance = rn->getRobot()->getOdometerDistance();
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
    rn->setIncrementPosition(deltaDistance, deltaDegrees);

    getRawPoseFromOdometry();
    //RNUtils::printLn("pose-Robot: {%f, %f, %f}, pose-Raw: {%f, %f, %f}, delta: {%f, %f}", robot->getPose().getX(), robot->getPose().getY(), robot->getPose().getThRad(), myRawPose->getX(), myRawPose->getY(), myRawPose->getThRad(), deltaDistance, deltaDegrees);
    prevVel = currentVel;
    prevRotVel = currentRotVel;
    prevDistance = currentDistance;
    prevRads = currentRads;

    rn->getRobot()->resetTripOdometer();
    
}

void RNFactorySensorsTask::getRawPoseFromOdometry(void){
    double x = 0, y = 0, th = 0;
    
    //th = myRawPose->getThRad() + deltaDegrees;
    ArPose *myRawPose = new ArPose(rn->getRobot()->getPose());
    x = myRawPose->getX() + deltaDistance * std::cos(myRawPose->getThRad() + (deltaDegrees / 2.0));
    y = myRawPose->getY() + deltaDistance * std::sin(myRawPose->getThRad() + (deltaDegrees / 2.0));
    th = myRawPose->getThRad() + deltaDegrees;
    
    delete myRawPose;
    myRawPose = NULL;

    rn->setPosition(x / 1000.0, y / 1000.0, th);

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
	
}