#include "RNDistanceTimerTask.h"

const unsigned int RNDistanceTimerTask::SECURITY_DISTANCE_WARNING_TIME = DEFAULT_SECURITY_DISTANCE_WARNING_TIME;
const unsigned int RNDistanceTimerTask::SECURITY_DISTANCE_STOP_TIME = DEFAULT_SECURITY_DISTANCE_STOP_TIME;

RNDistanceTimerTask::RNDistanceTimerTask(const RobotNode* rn, const char* name, const char* description) : RNRecurrentTask(rn, name, description){
	this->timerSecs = 0;
}

RNDistanceTimerTask::~RNDistanceTimerTask(){

}

void RNDistanceTimerTask::task(){
	if(rn->isGoalActive()){
		timerSecs++;
		RNUtils::sleep(987);
        if(this->timerSecs == this->SECURITY_DISTANCE_WARNING_TIME){
            rn->onSecurityDistanceWarningSignal();
        } else if(timerSecs == this->SECURITY_DISTANCE_STOP_TIME){
            rn->getRobot()->lock();
            rn->cancelGoal();
            rn->getRobot()->clearDirectMotion();
            rn->getRobot()->unlock();
            rn->notAllowedToMove(false);
            timerSecs = 0;
            rn->onSecurityDistanceStopSignal();
        }
    } else{
        timerSecs = 0;
    }
}

void RNDistanceTimerTask::onKilled(){
	
}