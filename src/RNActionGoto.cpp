#include "RNActionGoto.h"

RNActionGoto::RNActionGoto(RobotNode* rn, const char* name, ArPose goal, double linearSpeed, double angularSpeed, double minimumDistance, double minimumAngle) :
ArAction(name, "BP Implementation for turning and moving to a point"){
	this->rn = rn;
	this->actionDistance = SATURATION_DISTANCE_MM;
	this->actionDegrees = SATURATION_ANGLE_DEG;
	myDesired = new ArActionDesired;
	

	this->currentState = STATE_NO_GOAL;
	this->directionToTurn = 1;
	setNextArgument(ArArg("goal", &this->goal, "ArPose to go to. (ArPose)"));
	setGoal(goal);

	setNextArgument(ArArg("linear speed", &this->linearSpeed, "Linear Speed to travel to goal. (mm/sec)"));
	this->linearSpeed = linearSpeed;

	setNextArgument(ArArg("angular speed", &this->angularSpeed, "Angular Speed to point to goal. (degrees/sec)"));
	this->angularSpeed = angularSpeed;

	setNextArgument(ArArg("closest distance", &this->minimumDistance, "Distance that is close enough to goal. (mm)"));
	this->minimumDistance = minimumDistance;

	setNextArgument(ArArg("closest angle", &this->minimumDistance, "Angle that is close enough to point to goal. (degrees)"));
	this->minimumAngle = minimumAngle;

	linearController = new RNPIDController;
	angularController = new RNPIDController;
	hallwayController = new RNHallwayController;

	angularController->setSamplingTime(100);
	angularController->setProportionalGain(1);
	angularController->setDerivativeTime(this->minimumAngle / this->actionDegrees);

	linearController->setSamplingTime(100);
	linearController->setProportionalGain(.16);
	linearController->setDerivativeTime(this->minimumDistance / this->actionDistance);
}

RNActionGoto::~RNActionGoto(){
	delete myDesired;
	delete hallwayController;
	delete linearController;
	delete angularController;
}

ArActionDesired* RNActionGoto::fire(ArActionDesired current){
	if(this->currentState == STATE_GOING_TO_GOAL){

		double deltaThetaLocal, distanceLocal;
		ArPose* currPose = rn->getAltPose();
		//RNUtils::printLn("{currpose: {x: %f, y: %f, th: %f}}", currPose.getX(), currPose.getY(), currPose.getThRad());
		//RNUtils::printLn("{goal: {x: %f, y: %f, th: %f}}", goal.getX(), goal.getY(), goal.getThRad());
		if(this->goal.getTh() != 0.0){
			distanceLocal = 0.0;
			deltaThetaLocal = -currPose->getTh() + this->goal.getTh();
		} else {
			distanceLocal = currPose->findDistanceTo(this->goal);
	    	deltaThetaLocal = ArMath::subAngle(currPose->findAngleTo(this->goal), currPose->getTh());
	    }
    	//RNUtils::printLn("{Distance: %f, DeltaTheta: %f}", distanceLocal, deltaThetaLocal);
    	delete currPose;
		if(rn->isLaserReady() and (deltaThetaLocal > minimumAngle or distanceLocal > minimumDistance)){
			LaserScan* laserData = rn->getLaserScan();
			RNUtils::printLn("Laser size: %d", laserData->size());
			hallwayController->getSystemInput(laserData, &linearSpeed, &angularSpeed);
			RNUtils::printLn("{lin-vel: %f, rot-vel: %f}", linearSpeed, angularSpeed*180/M_PI);
			myDesired->setRotVel(angularSpeed*180/M_PI);
    		myDesired->setVel(linearSpeed);
		} else {
			myDesired->setVel(0);
    		myDesired->setRotVel(0);
    		angularController->reset();
    		linearController->reset();
    		currentState = STATE_ACHIEVED_GOAL;
		}

    	/*if(std::abs(deltaThetaLocal) > this->minimumAngle){
    		//turn to point to goal
    		myDesired->setVel(0);
    		linearController->reset();
    		double angVel = angularController->getSystemInput(deltaThetaLocal);
    		if(angVel >= angularSpeed){
    			angVel = -angularSpeed;
    		} else if(angVel <= -angularSpeed){
    			angVel = angularSpeed;
    		} else {
    			angVel *= -1;
    		}
    		//printf("{angVel after correction: %f}\n", angVel);

    		myDesired->setRotVel(angVel);
    		
    	} else if(distanceLocal > minimumDistance){
    		myDesired->setRotVel(0);
    		angularController->reset();
    		double linVel = linearController->getSystemInput(distanceLocal);
    		
    		if(linVel >= linearSpeed){
    			linVel = -linearSpeed;
    		} else if(linVel <= -linearSpeed){
    			linVel = linearSpeed;
    		} else {
    			linVel *= -1;
    		}
    		//printf("{linVel after correction: %f}\n", linVel);
    		myDesired->setVel(linVel);
    	} else {
    		myDesired->setVel(0);
    		myDesired->setRotVel(0);
    		angularController->reset();
    		linearController->reset();
    		RNUtils::printLn("Doris Stopped by this condition of goal achieved...");
    		currentState = STATE_ACHIEVED_GOAL;
    	}*/
    } else {
    	return NULL;
    }
	
    return myDesired;
}

bool RNActionGoto::haveAchievedGoal(void){
	return (this->currentState == STATE_ACHIEVED_GOAL);
}

bool RNActionGoto::haveCanceledGoal(void){
	return (this->currentState == STATE_NO_GOAL);
}

void RNActionGoto::cancelGoal(void){
	//myDesired->setVel(0);
	//myDesired->setRotVel(0);
	this->currentState = STATE_NO_GOAL;

}

void RNActionGoto::setGoal(ArPose goal, bool isHallway){
	this->isHallway = isHallway;
	this->previousGoal = this->goal;
	this->goal = goal;
	this->currentState = STATE_GOING_TO_GOAL;
}

double RNActionGoto::getLinearSpeed(void){
	return this->linearSpeed;
}

void RNActionGoto::setLinearSpeed(double speed){
	this->linearSpeed = speed;
}

double RNActionGoto::getAngularSpeed(void){
	return this->angularSpeed;
}

void RNActionGoto::setAngularSpeed(double speed){
	this->angularSpeed = speed;
}

double RNActionGoto::getMinimumDistance(void){
	return this->minimumDistance;
}

void RNActionGoto::setMinimumDistance(double distance){
	this->minimumDistance = distance;
	this->linearController->setProportionalGain(this->minimumDistance / this->actionDistance);
}

double RNActionGoto::getMinimumAngle(void){
	return this->minimumAngle;
}

void RNActionGoto::setMinimumAngle(double angle){
	this->minimumAngle = angle;
	this->angularController->setDerivativeTime(this->minimumAngle / this->actionDegrees);
}

ArPose RNActionGoto::getGoal(void){
	return this->goal;
}

double RNActionGoto::getActionDistance(void){
	return this->actionDistance;
}

void RNActionGoto::setActionDistance(double distance){
	this->actionDistance = distance;
	this->linearController->setProportionalGain(this->minimumDistance / this->actionDistance);
}

double RNActionGoto::getActionDegrees(void){
	return this->actionDegrees;
}

void RNActionGoto::setActionDegrees(double degrees){
	this->actionDegrees = degrees;
	this->angularController->setDerivativeTime(this->minimumAngle / this->actionDegrees);
}