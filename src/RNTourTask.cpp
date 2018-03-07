#include "RNTourTask.h"

RNTourTask::RNTourTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
}

RNTourTask::~RNTourTask(){

}

void RNTourTask::task(){

}

void RNTourTask::onKilled(){
	
}