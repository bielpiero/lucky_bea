#include "RNRecurrentTask.h"

RNRecurrentTask::RNRecurrentTask(const char* name, const char* description){

}

RNRecurrentTask::~RNRecurrentTask(){

}

void RNRecurrentTask::setController(GeneralController* gn){
	this->gn = gn;
}