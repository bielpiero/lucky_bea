#include "gaussBell.h"

namespace fuzzy{
	gaussBell::gaussBell(std::string name, float center, float width, float slope, float height){
		this->name = name;
		this->height = height;
		
		this->center = center;
		this->width = width;
		this->slope = slope;
	}
	
	gaussBell::~gaussBell(){
	
	}
	
	std::string gaussBell::className() const{
		return "gaussBell";
	}
	
	float gaussBell::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 1 / (1 - std::pow(std::abs((value - center)/width), 2 * slope));
		} 
		
		return (result * height);
	}
	
	gaussBell* gaussBell::operator+(const gaussBell& rhs) const{
		return new gaussBell("", this->center + rhs.center, this->width + rhs.width, this->slope + rhs.slope);
		
	}
	
	gaussBell* gaussBell::operator-(const gaussBell& rhs) const{
		return new gaussBell("", this->center - rhs.center, this->width - rhs.width, this->slope - rhs.slope);
	}
	
	gaussBell* gaussBell::operator*(const float& rhs) const{
		return new gaussBell("", this->center * rhs, this->width * rhs, this->slope * rhs);
	}
	
	gaussBell* gaussBell::clone() const{
		return new gaussBell(*this);
	}
	
	float gaussBell::getCenter() const{
		return center;
	}
	
	void gaussBell::setCenter(float value){
		this->center = value;
	}
	
	float gaussBell::getWidth() const{
		return width;
	}
	
	void gaussBell::setWidth(float value){
		this->width = value;
	}
	
	float gaussBell::getSlope() const{
		return slope;
	}
	
	void gaussBell::setSlope(float value){
		this->slope = value;
	}
}