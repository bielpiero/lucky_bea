#ifndef RN_LANDMARK_H
#define RN_LANDMARK_H

#include "RNUtils.h"

class RNLandmark{
private:
	int mapId;
	int sectorId;
	int markerId;
	std::vector<PointXYZ*>* data;
	double meanX;
	double meanY;
	double meanZ;

	std::vector<std::pair<std::string, double>* > *extras;
public:
	RNLandmark(){ 
		data = new std::vector<PointXYZ*>();
		extras = new std::vector<std::pair<std::string, double>* >();
		meanX = std::numeric_limits<double>::infinity();
		meanY = std::numeric_limits<double>::infinity();
		meanZ = std::numeric_limits<double>::infinity();
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
	void addPoint(double x, double y, double z = 0){
		meanX = std::numeric_limits<double>::infinity();
		meanY = std::numeric_limits<double>::infinity();
		meanZ = std::numeric_limits<double>::infinity();
		data->push_back(new PointXYZ(x, y, z));
	}

	void addExtraParameter(std::string name, double value){
		std::string realName = RNUtils::toLowercase(name);
		std::pair<std::string, double>* tuple = getExtraParameter(realName);
		if(tuple == NULL){
			extras->push_back(new std::pair<std::string, double>(realName, value));
		} else {
			tuple->second = value;
		}
	}

	std::pair<std::string, double>* getExtraParameter(std::string name){
		std::pair<std::string, double>* tuple = NULL;
		bool stop = false;
		for(int i = 0; i < extras->size() and not stop; i++){
			if(extras->at(i)->first == name){
				tuple = extras->at(i);
				stop = true;
			}
		}
		return tuple;
	}

	static RNLandmark* initializeFromString(char* landmarksString){
		RNLandmark* l = new RNLandmark();
		std::vector<std::string> values = RNUtils::split(landmarksString, ",");
		if(values.size() >4){
			l->mapId = std::atoi(values.at(0).c_str());
			l->sectorId = std::atoi(values.at(1).c_str());
			l->markerId = std::atoi(values.at(2).c_str());
			l->setPointsXMean(std::atof(values.at(3).c_str()));
			l->setPointsYMean(std::atof(values.at(4).c_str()));
		} else {
			delete l;
			l = NULL;
		}
		return l;
		
	}

	std::pair<std::string, double>* getExtraParameterAt(int index){
		return index > RN_NONE and index < extras->size() ? extras->at(index) : NULL;
	}

	PointXYZ* getPointAt(int index){
		return index > RN_NONE and index < data->size() ? data->at(index) : NULL;
	}

	void erasePointAt(int index){
		if(index > RN_NONE){
			delete data->at(index);
			data->erase(data->begin() + index);
		}
	}

	double getPointsXMean(){
		if(meanX == std::numeric_limits<double>::infinity()){
			double sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getX();
	        }
	        meanX = (sum / (double)data->size());
	    }
	    return meanX;
	}

	double getPointsYMean(){
		if(meanY == std::numeric_limits<double>::infinity()){
			double sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getY();
	        }
	        meanY = (sum / (double)data->size());
	    }
	    return meanY;
	}

	double getPointsZMean(){
		if(meanZ == std::numeric_limits<double>::infinity()){
			double sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getZ();
	        }
	        meanZ = (sum / (double)data->size());
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

	void setPointsXMean(double mean){
		this->meanX = mean;
	}

	void setPointsYMean(double mean){
		this->meanY = mean;
	}

	void setPointsZMean(double mean){
		this->meanZ = mean;
	}

	size_t size(){
		return data->size();
	}

	size_t extraParametersSize(){
		return extras->size();
	}
};

#endif