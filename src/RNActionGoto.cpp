#include "RNActionGoto.h"

RNActionGoto::RNActionGoto(const char* name, ArPose goal, double linearSpeed, double angularSpeed, double minimumDistance, double minimumAngle) :
ArAction(name, "BP Implementation for turning and moving to a point"){

}

RNActionGoto::~RNActionGoto(){

}

ArActionDesired* RNActionGoto::fire(ArActionDesired current){

}

bool RNActionGoto::haveAchievedGoal(void){
	return (this->currentState == STATE_ACHIEVED_GOAL);
}

bool RNActionGoto::haveCanceledGoal(void){
	return (this->currentState == STATE_NO_GOAL);
}

void RNActionGoto::cancelGoal(void){
	this->currentState = STATE_NO_GOAL;
}

void RNActionGoto::setGoal(ArPose goal){
	this->currentState = STATE_GOING_TO_GOAL;
	this->previousGoal = this->goal;
	this->goal = goal;
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
}

double RNActionGoto::getMinimumAngle(void){
	return this->minimumAngle;
}

void RNActionGoto::setMinimumAngle(double angle){
	this->minimumAngle = angle;
}