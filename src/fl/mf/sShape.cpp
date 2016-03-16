#include "sShape.h"

namespace fuzzy{
	sShape::sShape(std::string name, float start, float end, float height){
		this->name = name;
		this->height = height;
		this->start = start;
		this->end = end;
	}
	
	sShape::~sShape(){
	
	}
	
	std::string sShape::className() const{
		return "sShape";
	}
	
	float sShape::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 0;
			if(value <= start){
				result = 0.0;
			} else if(value >= end){
				result = 1.0;
			} else if((value >= start) && (value <= ((start + end) / 2))){
				result = 2 * std::pow(((value - start)/(end - start)),2);
			} else if((value <= end) && (value >= ((start + end) / 2))){
				result = 1 - 2 * std::pow(((value - start)/(end - start)),2);
			}
		} 
		
		return (result * height);
	}
	
	sShape* sShape::operator+(const sShape& rhs) const{
		return new sShape("", this->start + rhs.start, this->end + rhs.end);
		
	}
	
	sShape* sShape::operator-(const sShape& rhs) const{
		return new sShape("", this->start - rhs.start, this->end - rhs.end);
	}
	
	sShape* sShape::operator*(const float& rhs) const{
		return new sShape("", this->start * rhs, this->end * rhs);
	}
	
	sShape* sShape::clone() const{
		return new sShape(*this);
	}
	
	void sShape::setStart(float start) {
        this->start = start;
    }
	
	float sShape::getStart() const {
        return this->start;
    }

    void sShape::setEnd(float end) {
        this->end = end;
    }

    float sShape::getEnd() const {
        return this->end;
    }
}
