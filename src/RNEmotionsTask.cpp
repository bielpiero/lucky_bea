#include "RNEmotionsTask.h"

RNEmotionsTask::RNEmotionsTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
}

RNEmotionsTask::~RNEmotionsTask(){

}

void RNEmotionsTask::task(){

}

void RNEmotionsTask::onKilled(){
	
}