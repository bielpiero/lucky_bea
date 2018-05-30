#include "RNPFLocalizationTask.h"

RNPFLocalizationTask::RNPFLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description){
	enableLocalization = false;
}

RNPFLocalizationTask::~RNPFLocalizationTask(){

}

void RNPFLocalizationTask::init(){
	if(gn != NULL){
		//laserLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		//cameraLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);
		//rfidLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);

		alpha = 0.2;
		enableLocalization = true;

	} else{
		enableLocalization = false;
	}
}

void RNPFLocalizationTask::onKilled(){
	enableLocalization = false;
}

void RNPFLocalizationTask::task(){
	if(enableLocalization){
		
	} else {
		init();
	}
	RNUtils::sleep(10);
}