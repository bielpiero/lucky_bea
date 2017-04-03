#include "sShape.h"

namespace fuzzy{
	SShape::SShape(std::string name, float start, float end, float height){
		this->name = name;
		this->height = height;
		this->start = start;
		this->end = end;
	}
	
	SShape::~SShape(){
	
	}
	
	std::string SShape::className() const{
		return "SShape";
	}
	
	float SShape::evaluate(float value) const{
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
	
	SShape* SShape::operator+(const SShape& rhs) const{
		return new SShape("", this->start + rhs.start, this->end + rhs.end);
		
	}
	
	SShape* SShape::operator-(const SShape& rhs) const{
		return new SShape("", this->start - rhs.start, this->end - rhs.end);
	}
	
	SShape* SShape::operator*(const float& rhs) const{
		return new SShape("", this->start * rhs, this->end * rhs);
	}
	
	SShape* SShape::clone() const{
		return new SShape(*this);
	}
	
	void SShape::setStart(float start) {
        this->start = start;
    }
	
	float SShape::getStart() const {
        return this->start;
    }

    void SShape::setEnd(float end) {
        this->end = end;
    }

    float SShape::getEnd() const {
        return this->end;
    }
}
