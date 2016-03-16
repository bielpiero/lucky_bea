#include "sigmoid.h"

namespace fuzzy{
	sigmoid::sigmoid(std::string name, float inflection, float slope, float height){
		this->name = name;
		this->height = height;
		
		this->inflection= inflection;
		this->slope = slope;
	}
	
	sigmoid::~sigmoid(){
	
	}
	
	std::string sigmoid::className() const{
		return "sigmoid";
	}
	
	float sigmoid::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 1 / (1 + std::exp(-slope * (value - inflection)));
		} 
		return (result * height);
	}
	
	sigmoid* sigmoid::clone() const{
		return new sigmoid(*this);
	}
	
	sigmoid* sigmoid::operator+(const sigmoid& rhs) const{
		return new sigmoid("", this->inflection+ rhs.inflection, this->slope + rhs.slope);
		
	}
	
	sigmoid* sigmoid::operator-(const sigmoid& rhs) const{
		return new sigmoid("", this->inflection - rhs.inflection, this->slope - rhs.slope);
	}
	
	sigmoid* sigmoid::operator*(const float& rhs) const{
		return new sigmoid("", this->inflection * rhs, this->slope * rhs);
	}
	
	void sigmoid::setInflection(float value) {
        this->inflection = value;
    }

    float sigmoid::getInflection() const {
        return this->inflection;
    }

    void sigmoid::setSlope(float value) {
        this->slope = value;
    }

    float sigmoid::getSlope() const {
		return this->slope;
    }

}