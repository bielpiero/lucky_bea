#ifndef ANTENNA_DATA_LIST_H
#define ANTENNA_DATA_LIST_H

#include "AntennaData.h"
#include <iostream>
#include <vector>

class AntennaDataList{
public:
	AntennaDataList(){
		antennas = new std::vector<AntennaData*>();
	}

	~AntennaDataList(){
		clear();
		delete antennas;
	}

	int add(AntennaData* ant){
		int result = 0;
		if(find(ant->getAntennaId()) == NULL){
			antennas->push_back(ant);
		} else {
			result = -1;
		}
		return result;
	}

	int remove(int id){
		int result = 0;
		int index = 0;
		if(find(id, index) != NULL){
			antennas->erase(antennas->begin() + index);
		} else {
			result = -1;
		}
		return result;
	}

	AntennaData* find(int id, int index = -1){
		AntennaData* result = NULL;
		bool found = false;
		for(int i = 0; i < antennas->size() and not found; i++){
			if(antennas->at(i)->getAntennaId() == id){
				found = true;
				index = i;
				result = antennas->at(i);
			}
		}
		return result;
	}

	void clear(){
		for (int i = 0; i < antennas->size(); i++){
			delete antennas->at(i);
		}
		antennas->clear();
	}

private:
	std::vector<AntennaData*>* antennas;
};

#endif