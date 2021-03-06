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

	//linearController = new RNPIDController("Linear Speed", 0, 30, -0.16, 0, 0, 1, -linearSpeed, linearSpeed);
	//angularController = new RNPIDController("Angular Speed", 0, 30, -0.45, 0, 0, 1, -angularSpeed, angularSpeed);
	//speedController = new RNFuzzySpeedController;
	//hallwayController = new RNHallwayController;

	/*montse*/
	speedControlDNC = new RNFuzzySpeedControl;
	dncController = new RNFuzzyDNC;
	hallController = new RNFuzzyHallDNC;
	/*montse*/
}

RNActionGoto::~RNActionGoto(){
	delete myDesired;
	//delete speedController;
	/*montse*/
	delete hallController;
	delete dncController;
	delete speedControlDNC;
	/*montse*/
	//delete hallwayController;
	//delete linearController;
	//delete angularController;
}

ArActionDesired* RNActionGoto::fire(ArActionDesired current){
	if(this->currentState == STATE_GOING_TO_GOAL){

		float deltaThetaLocal = 0.0, distanceLocal = 0.0;
		//ArPose* currPose = new ArPose(myRobot->getPose());
		ArPose* currPose = rn->getAltPose();
		LaserScan* laserData = rn->getLaserScan();

		distanceLocal = RNUtils::distanceTo(this->goal.getX(), this->goal.getY(), currPose->getX(), currPose->getY());

		/* VARIABLE PARA INDICAR SI ES PASILLO */
		//isHallway ...

		//RNUtils::printLn("{currpose: {x: %f, y: %f, th: %f}}", currPose.getX(), currPose.getY(), currPose.getThRad());
		//RNUtils::printLn("{goal: {x: %f, y: %f, th: %f}}", goal.getX(), goal.getY(), goal.getThRad());
		if(currPose){
			if(isHallway){
				deltaThetaLocal = hallController->getDeltaTheta(laserData, currPose, &goal);
				//RNUtils::printLn("DNC: PASILLO");
			}
			else{
				deltaThetaLocal = dncController->getDeltaTheta(laserData, currPose, &goal);
				//RNUtils::printLn("DNC: SALA");
			}
			//RNUtils::printLn("{FuzzyDNC DeltaTheta: %f}", deltaThetaLocal);
	    	//RNUtils::printLn("{Distance: %f, DeltaTheta: %f}", distanceLocal, deltaThetaLocal);
	    	delete currPose;
	    }

		if((deltaThetaLocal > minimumAngle or deltaThetaLocal < -minimumAngle)){
			//Funcion de rotar... preguntar a Biel si separar rotacion de avance?
			myDesired->setRotVel(0.1); //Consultar rango
    		myDesired->setVel(0);
		}
		//Go forward
    	else if((distanceLocal > minimumDistance) or (distanceLocal < -minimumDistance)){
			int i = 0;
			//Get closest obstacle to the LEFT (sectors 0 to 160)
			float obsLeft = 100;
			for(i=0; i<((laserData->size()/2)-20);i++){
				if(laserData->getRange(i)<obsLeft) obsLeft =laserData->getRange(i);
			}

			//Get closest obstable to the RIGHT (sectors 200 to 360)
			float obsRight = 100;
			for(i=((laserData->size()/2)+20); i<laserData->size();i++){
				if(laserData->getRange(i)<obsRight) obsRight =laserData->getRange(i);
			}

			//Get cloaset obstacle to the FRONT (sectors 160 to 200)
			float obsFront = 100;
			for(i=((laserData->size()/2)-20); i<((laserData->size()/2)+20);i++){
				if(laserData->getRange(i)<obsFront) obsFront =laserData->getRange(i);
			}

			speedControlDNC->getSystemInput(obsLeft, obsRight, obsFront, &linearSpeed, &angularSpeed);

    		//speedControl->getSystemInput(distanceLocal/1e3, RNUtils::deg2Rad(deltaThetaLocal), &linearSpeed, &angularSpeed);
    		//RNUtils::printLn("{lin-vel: %f, rot-vel: %f}", linearSpeed, RNUtils::rad2Deg(angularSpeed));
			myDesired->setRotVel(0);
    		myDesired->setVel(linearSpeed);
    	} else {
    		myDesired->setVel(0);
    		myDesired->setRotVel(0);
    		currentState = STATE_ACHIEVED_GOAL;
    	}
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