#include "sigmoid.h"

namespace fuzzy{
	Sigmoid::Sigmoid(std::string name, float inflection, float slope, float height){
		this->name = name;
		this->height = height;
		
		this->inflection= inflection;
		this->slope = slope;
	}
	
	Sigmoid::~Sigmoid(){
	
	}
	
	std::string Sigmoid::className() const{
		return "Sigmoid";
	}
	
	float Sigmoid::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 1 / (1 + std::exp(-slope * (value - inflection)));
		} 
		return (result * height);
	}
	
	Sigmoid* Sigmoid::clone() const{
		return new Sigmoid(*this);
	}
	
	Sigmoid* Sigmoid::operator+(const Sigmoid& rhs) const{
		return new Sigmoid("", this->inflection+ rhs.inflection, this->slope + rhs.slope);
		
	}
	
	Sigmoid* Sigmoid::operator-(const Sigmoid& rhs) const{
		return new Sigmoid("", this->inflection - rhs.inflection, this->slope - rhs.slope);
	}
	
	Sigmoid* Sigmoid::operator*(const float& rhs) const{
		return new Sigmoid("", this->inflection * rhs, this->slope * rhs);
	}
	
	void Sigmoid::setInflection(float value) {
        this->inflection = value;
    }

    float Sigmoid::getInflection() const {
        return this->inflection;
    }

    void Sigmoid::setSlope(float value) {
        this->slope = value;
    }

    float Sigmoid::getSlope() const {
		return this->slope;
    }

}