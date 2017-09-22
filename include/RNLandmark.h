#ifndef RN_LANDMARK_H
#define RN_LANDMARK_H

#include "RNUtils.h"

class RNLandmark{
private:
	int mapId;
	int sectorId;
	int markerId;
	std::vector<PointXY*>* data;
	float meanX;
	float meanY;
public:
	RNLandmark(){ 
		data = new std::vector<PointXY*>(); 
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
	}

	~RNLandmark() {
		for (int i = 0; i < data->size(); i++) {
            delete data->at(i);
        }
		data->clear();
		delete data; 
	}
	void addPoint(float x, float y){
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
		data->push_back(new PointXY(x, y));
	}

	PointXY* getPointAt(int index){
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

	size_t size(){
		return data->size();
	}
};

#endif