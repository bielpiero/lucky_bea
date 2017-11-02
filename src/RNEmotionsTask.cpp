#include "RNEmotionsTask.h"

RNEmotionsTask::RNEmotionsTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	this->gn = (GeneralController*)rn;
}

RNEmotionsTask::~RNEmotionsTask(){

}

void RNEmotionsTask::task(){

}

void RNEmotionsTask::onKilled(){
	
}