#include "mf.h"

namespace fuzzy{
	MF::MF(const std::string name, float height){
		this->name = name;
		this->height = height;
	}
	
	MF::~MF(){
	}
	
	std::string MF::getName() const{
		return this->name;
	}
	
	void MF::setName(std::string name){
		this->name = name;
	}
		
	float MF::getHeight() const{
		return this->height;
	}

	MF* MF::operator+(const MF &rhs) const{
	}

	MF* MF::operator-(const MF &rhs) const{
	}
	
	MF* MF::operator*(const float& rhs) const{
	}
	
	void MF::setHeight(float height){
		this->height = height;
	}
}
