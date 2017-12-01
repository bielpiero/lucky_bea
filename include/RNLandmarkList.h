#ifndef RN_LANDMARK_LIST_H
#define RN_LANDMARK_LIST_H

#include "RNLandmark.h"

class RNLandmarkList{
private:
	std::vector<RNLandmark*> *landmarks;
public:
	RNLandmarkList(){
		landmarks = new std::vector<RNLandmark*>();
	}

	~RNLandmarkList(){
		clear();
		delete landmarks;

	}

	int add(RNLandmark* landmark){
		int result = RN_NONE;
		if(landmark != NULL){
			landmarks->push_back(landmark);
			result = 0;
		}
		return result;
	}

	RNLandmark* at(int index){
		return landmarks->at(index);
	}

	RNLandmark* find(int mapId, int sectorId, int id){
		RNLandmark* result = NULL;
		bool found = false;
		for(int i = 0; i < landmarks->size() and not found; i++){
			if(landmarks->at(i)->getMapId() == mapId and landmarks->at(i)->getSectorId() == sectorId and landmarks->at(i)->getMarkerId() == id){
				result = landmarks->at(i);
				found = true;
			}
		}
		return result;
	}

	int indexOf(RNLandmark* landmark){
		bool found = false;
		int index = RN_NONE;
		for(int i = 0; i < landmarks->size() and not found; i++){
			if(landmarks->at(i)->isEqualTo(landmark)){
				index = i;
				found = true;
			}
		}
		return index;
	}

	int remove(int mapId, int sectorId, int id){
		int index = indexOf(find(mapId, sectorId, id));
		if(index > RN_NONE){
			delete landmarks->at(index);
			landmarks->erase(landmarks->begin() + index);
		}
	}

	int size(){
		return landmarks->size();
	}

	void clear(){
		for(int i = 0; i < landmarks->size(); i++){
			delete landmarks->at(i);
		}
		landmarks->clear();
	}

	const char* toString() const{
		std::ostringstream print_str;
		print_str.str("");
		print_str.clear();
		print_str << "\"Landmarks\": {\"Landmark\": [";
		for(int i = 0; i < landmarks->size(); i++){
			print_str << "{";
			print_str << "\"Map-Id\": \"" << landmarks->at(i)->getMapId() << "\",";
			print_str << "\"Sector-Id\": \"" << landmarks->at(i)->getSectorId() << "\",";
			print_str << "\"Landmark-Id\": \"" << landmarks->at(i)->getMarkerId() << "\",";
			print_str << "\"distance\": \"" << landmarks->at(i)->getPointsXMean() << "\",";
			print_str << "\"angle\": \"" << landmarks->at(i)->getPointsYMean() << "\"";
			if(landmarks->at(i)->extraParametersSize() > 0){
				print_str << ",";
			}
			for(int j = 0; j < landmarks->at(i)->extraParametersSize(); j++){
				std::pair<std::string, float>* tuple = landmarks->at(i)->getExtraParameterAt(j);
				print_str << "\"" << tuple->first << "\": \"" << tuple->second << "\"";
				if(j < landmarks->at(i)->extraParametersSize() - 1){
					print_str << ",";
				}
			}
			print_str << "}";

			if(i < landmarks->size() - 1){
				print_str << ",";
			}
		}
		print_str << "]}";
		std::string strprint = print_str.str();
		return strprint.c_str();
	}
};

#endif