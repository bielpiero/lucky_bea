#include "rule.h"

namespace fuzzy{
	rule::rule(const std::string name, float weight){
		this->name = name;
		this->weight = weight;
	}
	
	rule::~rule(){
	
	}
	
	void rule::setName(const std::string name){
		this->name = name;
    }

    std::string rule::getName() const{
        return this->name;
    }
	
	void rule::setWeight(float value){
		this->weight = value;
    }

    float rule::getWeight() const{
        return this->weight;
    }
}