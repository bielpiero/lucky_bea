#include "piShape.h"

namespace fuzzy{
	piShape::piShape(std::string name, float bottomLeft, float topLeft, float bottomRight, float topRight, float height){
		this->name = name;
		this->height = height;
		
		this->bottomLeft = bottomLeft;
		this->topLeft = topLeft;
		
		this->bottomRight = bottomRight;
		this->topRight = topRight;
	}
	
	piShape::~piShape(){
	
	}
	
	std::string piShape::className() const{
		return "piShape";
	}
	
	float piShape::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			sShape* s = new sShape("", bottomLeft, topLeft);
			zShape* z = new zShape("", topRight, bottomRight);
			float f1 = s->evaluate(value);
			float f2 = z->evaluate(value);
			result = f1 * f2;
		} 
		return (result * height);
	}
	
	piShape* piShape::clone() const{
		return new piShape(*this);
	}
	
	float piShape::getBottomLeft() const{
		return bottomLeft;
	}
	
	void piShape::setBottomLeft(float value){
		this->bottomLeft = value;
	}
	
	float piShape::getTopLeft() const{
		return topLeft;
	}
	
	void piShape::setTopLeft(float value){
		this->topLeft = value;
	}
	
	float piShape::getBottomRight() const{
		return bottomRight;
	}
	
	void piShape::setBottomRight(float value){
		this->bottomRight = value;
	}
	
	float piShape::getTopRight() const{
		return topRight;
	}
	
	void piShape::setTopRight(float value){
		this->topRight = value;
	}
}