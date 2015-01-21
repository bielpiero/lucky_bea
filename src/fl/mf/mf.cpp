#include "mf.h"

namespace fuzzy{
	mf::mf(const std::string name, float height){
		this->name = name;
		this->height = height;
	}
	
	mf::~mf(){
	}
	
	std::string mf::getName() const{
		return this->name;
	}
	
	void mf::setName(std::string name){
		this->name = name;
	}
		
	float mf::getHeight() const{
		return this->height;
	}
	
	void mf::setHeight(float height){
		this->height = height;
	}
}