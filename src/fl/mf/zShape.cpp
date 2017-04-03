#include "zShape.h"

namespace fuzzy{
	ZShape::ZShape(std::string name, float start, float end, float height){
		this->name = name;
		this->height = height;
		this->start = start;
		this->end = end;
	}
	
	ZShape::~ZShape(){
	
	}
	
	std::string ZShape::className() const{
		return "ZShape";
	}
	
	float ZShape::evaluate(float value) const{
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
	
	ZShape* ZShape::operator+(const ZShape& rhs) const{
		return new ZShape("", this->start + rhs.start, this->end + rhs.end);
		
	}
	
	ZShape* ZShape::operator-(const ZShape& rhs) const{
		return new ZShape("", this->start - rhs.start, this->end - rhs.end);
	}
	
	ZShape* ZShape::operator*(const float& rhs) const{
		return new ZShape("", this->start * rhs, this->end * rhs);
	}
	
	ZShape* ZShape::clone() const{
		return new ZShape(*this);
	}
	
	void ZShape::setStart(float start) {
        this->start = start;
    }
	
	float ZShape::getStart() const {
        return this->start;
    }

    void ZShape::setEnd(float end) {
        this->end = end;
    }

    float ZShape::getEnd() const {
        return this->end;
    }
}