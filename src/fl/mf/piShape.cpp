#include "piShape.h"

namespace fuzzy{
	PiShape::PiShape(std::string name, float bottomLeft, float topLeft, float bottomRight, float topRight, float height){
		this->name = name;
		this->height = height;
		
		this->bottomLeft = bottomLeft;
		this->topLeft = topLeft;
		
		this->bottomRight = bottomRight;
		this->topRight = topRight;
	}
	
	PiShape::~PiShape(){
	
	}
	
	std::string PiShape::className() const{
		return "PiShape";
	}
	
	float PiShape::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			SShape* s = new SShape("", bottomLeft, topLeft);
			ZShape* z = new ZShape("", topRight, bottomRight);
			float f1 = s->evaluate(value);
			float f2 = z->evaluate(value);
			result = f1 * f2;
		} 
		return (result * height);
	}
	
	PiShape* PiShape::operator+(const PiShape& rhs) const{
		return new PiShape("", this->bottomLeft + rhs.bottomLeft, this->topLeft + rhs.topLeft, this->bottomRight + rhs.bottomRight, this->topRight + rhs.topRight);
		
	}
	
	PiShape* PiShape::operator-(const PiShape& rhs) const{
		return new PiShape("", this->bottomLeft - rhs.bottomLeft, this->topLeft - rhs.topLeft, this->bottomRight - rhs.bottomRight, this->topRight - rhs.topRight);
	}
	
	PiShape* PiShape::operator*(const float& rhs) const{
		return new PiShape("", this->bottomLeft * rhs, this->topLeft * rhs, this->bottomRight * rhs, this->topRight * rhs);
	}
	
	PiShape* PiShape::clone() const{
		return new PiShape(*this);
	}
	
	float PiShape::getBottomLeft() const{
		return bottomLeft;
	}
	
	void PiShape::setBottomLeft(float value){
		this->bottomLeft = value;
	}
	
	float PiShape::getTopLeft() const{
		return topLeft;
	}
	
	void PiShape::setTopLeft(float value){
		this->topLeft = value;
	}
	
	float PiShape::getBottomRight() const{
		return bottomRight;
	}
	
	void PiShape::setBottomRight(float value){
		this->bottomRight = value;
	}
	
	float PiShape::getTopRight() const{
		return topRight;
	}
	
	void PiShape::setTopRight(float value){
		this->topRight = value;
	}
}