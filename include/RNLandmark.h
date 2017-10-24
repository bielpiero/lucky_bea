#ifndef RN_LANDMARK_H
#define RN_LANDMARK_H

#include "RNUtils.h"

class RNLandmark{
private:
	int mapId;
	int sectorId;
	int markerId;
	std::vector<PointXYZ*>* data;
	float meanX;
	float meanY;
	float meanZ;

	std::vector<std::pair<std::string, float>* > *extras;
public:
	RNLandmark(){ 
		data = new std::vector<PointXYZ*>();
		extras = new std::vector<std::pair<std::string, float>* >();
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
		meanZ = std::numeric_limits<float>::infinity();
	}

	~RNLandmark() {
		for (int i = 0; i < data->size(); i++) {
            delete data->at(i);
        }
		data->clear();
		delete data;

		for (int i = 0; i < extras->size(); i++) {
            delete extras->at(i);
        }
		extras->clear();
		delete extras;
	}
	void addPoint(float x, float y, float z = 0){
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
		meanZ = std::numeric_limits<float>::infinity();
		data->push_back(new PointXYZ(x, y, z));
	}

	void addExtraParameter(std::string name, float value){
		std::string realName = RNUtils::toLowercase(name);
		std::pair<std::string, float>* tuple = getExtraParameter(realName);
		if(tuple == NULL){
			extras->push_back(new std::pair<std::string, float>(realName, value));
		} else {
			tuple->second = value;
		}
	}

	std::pair<std::string, float>* getExtraParameter(std::string name){
		std::pair<std::string, float>* tuple = NULL;
		bool stop = false;
		for(int i = 0; i < extras->size() and not stop; i++){
			if(extras->at(i)->first == name){
				tuple = extras->at(i);
				stop = true;
			}
		}
		return tuple;
	}

	PointXYZ* getPointAt(int index){
		return data->at(index);
	}

	void erasePointAt(int index){
		if(index > RN_NONE){
			delete data->at(index);
			data->erase(data->begin() + index);
		}
	}

	float getPointsXMean(){
		if(meanX == std::numeric_limits<float>::infinity()){
			float sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getX();
	        }
	        meanX = (sum / (float)data->size());
	    }
	    return meanX;
	}

	float getPointsYMean(){
		if(meanY == std::numeric_limits<float>::infinity()){
			float sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getY();
	        }
	        meanY = (sum / (float)data->size());
	    }
	    return meanY;
	}

	float getPointsZMean(){
		if(meanZ == std::numeric_limits<float>::infinity()){
			float sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getZ();
	        }
	        meanZ = (sum / (float)data->size());
	    }
	    return meanZ;
	}

	bool isEqualTo(RNLandmark* landmark){
		bool result = false;
		if(landmark != NULL){
			if(this->getMapId() == landmark->getMapId() and this->getSectorId() == landmark->getSectorId() and this->getMarkerId() == landmark->getMarkerId()){
				result = true;
			}	
		}
		return result;
	}

	void setMapId(int id){
		this->mapId = id;
	}

	void setSectorId(int id){
		this->sectorId = id;
	}

	void setMarkerId(int id){
		this->markerId = id;
	}

	int getMapId(){
		return mapId;
	}

	int getMarkerId(){
		return markerId;
	}

	int getSectorId(){
		return sectorId;
	}

	void setPointsXMean(float mean){
		this->meanX = mean;
	}

	void setPointsYMean(float mean){
		this->meanY = mean;
	}

	void setPointsZMean(float mean){
		this->meanZ = mean;
	}

	size_t size(){
		return data->size();
	}
};

#endif