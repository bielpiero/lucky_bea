#include "trapezoid.h"

namespace fuzzy{
	trapezoid::trapezoid(std::string name, float vertexA, float vertexB, float vertexC, float vertexD, float height){
		this->name = name;
		this->vertexA = vertexA;
		this->vertexB = vertexB;
		this->vertexC = vertexC;
		this->vertexD = vertexD;
		this->height = height;
	}
	
	trapezoid::~trapezoid(){
	
	}
	
	std::string trapezoid::className() const{
		return "trapezoid";
	}
	
	float trapezoid::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 0;
			if((vertexA == -fuzzy::inf) && (value <= vertexB)){
				result = 1.0;
			} else if((value > vertexC) && (vertexD == fuzzy::inf)){
				result = 1.0;
			} else {
				std::vector<float> minv(3);
				minv[0] = (value-vertexA)/(vertexB-vertexA);
				minv[1] = 1;
				minv[2] = (vertexD-value)/(vertexD-vertexC);
				
				std::vector<float> maxv(2);
				maxv[0] = stats::min(minv);
				maxv[1] = 0;
				
				result = stats::max(maxv);
			}
		} 
		
		return (result * height);
	}
	
	trapezoid* trapezoid::clone() const{
		return new trapezoid(*this);
	}
	
	trapezoid* trapezoid::operator+(const trapezoid* rhs) const{
		return new trapezoid("", this->vertexA + rhs->vertexA, this->vertexB + rhs->vertexB, this->vertexC + rhs->vertexC, this->vertexD + rhs->vertexD);
		
	}
	
	trapezoid* trapezoid::operator-(const trapezoid* rhs) const{
		return new trapezoid("", this->vertexA - rhs->vertexA, this->vertexB - rhs->vertexB, this->vertexC - rhs->vertexC, this->vertexD - rhs->vertexD);
	}
	
	trapezoid* trapezoid::operator*(const float& rhs) const{
		return new trapezoid("", this->vertexA * rhs, this->vertexB * rhs, this->vertexC * rhs, this->vertexD * rhs);
	}
	
	float trapezoid::getVertexA() const{
		return vertexA;
	}
	
	void trapezoid::setVertexA(float value){
		this->vertexA = value;
	}
	
	float trapezoid::getVertexB() const{
		return vertexB;
	}
	
	void trapezoid::setVertexB(float value){
		this->vertexB = value;
	}
	
	float trapezoid::getVertexC() const{
		return vertexC;
	}
	
	void trapezoid::setVertexC(float value){
		this->vertexC = value;
	}
	
	float trapezoid::getVertexD() const{
		return vertexD;
	}
	
	void trapezoid::setVertexD(float value){
		this->vertexD = value;
	}
	
}