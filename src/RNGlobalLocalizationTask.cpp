#include "RNGlobalLocalizationTask.h"

RNGlobalLocalizationTask::RNGlobalLocalizationTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	enableGlobalLocalization = false;
	//history = new std::vector<Trio<int, int, int> >();
}

RNGlobalLocalizationTask::~RNGlobalLocalizationTask(){

}

void RNGlobalLocalizationTask::onKilled(){
	enableGlobalLocalization = false;
}

void RNGlobalLocalizationTask::task(){
	std::vector<Trio<int, int, int> > history;
	Trio<int, int, int> firstSector(RN_NONE, RN_NONE, RN_NONE);
	Trio<int, int, int> secondSector(RN_NONE, RN_NONE, RN_NONE);
	RNLandmarkList* markers = new RNLandmarkList(*gn->getVisualLandmarks());
	for(int i = 0; i < markers->size(); i++){
		bool found = false;
		int index = RN_NONE;
		for(int j = 0; j < history.size() and not found; j++){
			if(history.at(j).getFirst() == markers->at(i)->getMapId() and history.at(j).getSecond() == markers->at(i)->getSectorId()){
				found = true;
				index = j;
			}
		}
		if(index != RN_NONE){
			history.at(index).setThird(history.at(index).getThird() + 1);
		} else {
			history.push_back(Trio<int, int, int>(markers->at(i)->getMapId(), markers->at(i)->getSectorId(), 1));
		}
	}
	//markers->clear();
	//delete markers;
	for(int i = 0; i < history.size(); i++){
		if(firstSector.getThird() <= history.at(i).getThird()){
			secondSector = firstSector;
			firstSector = history.at(i);
		}
	}
	RNUtils::printLn("firstSector: {M: %d, S: %d, H: %d}", firstSector.getFirst(), firstSector.getSecond(), firstSector.getThird());
	RNUtils::printLn("secondSector: {M: %d, S: %d, H: %d}", secondSector.getFirst(), secondSector.getSecond(), secondSector.getThird());
	/*if(enableGlobalLocalization){
		
	} else {
		
	}*/
	RNUtils::sleep(10);
}