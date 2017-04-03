#include "gaussBell.h"

namespace fuzzy{
	GaussBell::GaussBell(std::string name, float center, float width, float slope, float height){
		this->name = name;
		this->height = height;
		
		this->center = center;
		this->width = width;
		this->slope = slope;
	}
	
	GaussBell::~GaussBell(){
	
	}
	
	std::string GaussBell::className() const{
		return "GaussBell";
	}
	
	float GaussBell::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 1 / (1 - std::pow(std::abs((value - center)/width), 2 * slope));
		} 
		
		return (result * height);
	}
	
	GaussBell* GaussBell::operator+(const GaussBell& rhs) const{
		return new GaussBell("", this->center + rhs.center, this->width + rhs.width, this->slope + rhs.slope);
		
	}
	
	GaussBell* GaussBell::operator-(const GaussBell& rhs) const{
		return new GaussBell("", this->center - rhs.center, this->width - rhs.width, this->slope - rhs.slope);
	}
	
	GaussBell* GaussBell::operator*(const float& rhs) const{
		return new GaussBell("", this->center * rhs, this->width * rhs, this->slope * rhs);
	}
	
	GaussBell* GaussBell::clone() const{
		return new GaussBell(*this);
	}
	
	float GaussBell::getCenter() const{
		return center;
	}
	
	void GaussBell::setCenter(float value){
		this->center = value;
	}
	
	float GaussBell::getWidth() const{
		return width;
	}
	
	void GaussBell::setWidth(float value){
		this->width = value;
	}
	
	float GaussBell::getSlope() const{
		return slope;
	}
	
	void GaussBell::setSlope(float value){
		this->slope = value;
	}
}