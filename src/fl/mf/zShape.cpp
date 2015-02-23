#include "zShape.h"

namespace fuzzy{
	zShape::zShape(std::string name, float start, float end, float height){
		this->name = name;
		this->height = height;
		this->start = start;
		this->end = end;
	}
	
	zShape::~zShape(){
	
	}
	
	std::string zShape::className() const{
		return "zShape";
	}
	
	float zShape::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 0;
			if(value <= start){
				result = 1.0;
			} else if(value >= end){
				result = 0.0;
			} else if((value >= start) && (value <= ((start + end) / 2))){
				result = 1 - 2 * std::pow(((value - start)/(end - start)),2);
			} else if((value <= end) && (value >= ((start + end) / 2))){
				result = 2 * std::pow(((value - start)/(end - start)),2);
			}
			
		}
		return (result * height);
	}
	
	zShape* zShape::operator+(const zShape* rhs) const{
		return new zShape("", this->start + rhs->start, this->end + rhs->end);
		
	}
	
	zShape* zShape::operator-(const zShape* rhs) const{
		return new zShape("", this->start - rhs->start, this->end - rhs->end);
	}
	
	zShape* zShape::operator*(const float& rhs) const{
		return new zShape("", this->start * rhs, this->end * rhs);
	}
	
	zShape* zShape::clone() const{
		return new zShape(*this);
	}
	
	void zShape::setStart(float start) {
        this->start = start;
    }
	
	float zShape::getStart() const {
        return this->start;
    }

    void zShape::setEnd(float end) {
        this->end = end;
    }

    float zShape::getEnd() const {
        return this->end;
    }
}