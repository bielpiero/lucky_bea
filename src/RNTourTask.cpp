#include "RNTourTask.h"

RNTourTask::RNTourTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	initialized = false;
	lastSiteVisitedIndex = RN_NONE;
}

RNTourTask::~RNTourTask(){

}

void RNTourTask::task(){
	if(initialized){

	} else {
		init();
	}
}

void RNTourTask::onKilled(){
	initialized = false;
	lastSiteVisitedIndex = RN_NONE;
}

void RNTourTask::init(){
	if(gn != NULL){
		sequence = RNUtils::split(gn->getCurrentSector()->getSequence(), ",");
		initialized = true;
	}
}