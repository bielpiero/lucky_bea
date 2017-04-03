#include "trapezoid.h"

namespace fuzzy{
	Trapezoid::Trapezoid(std::string name, float vertexA, float vertexB, float vertexC, float vertexD, float height){
		this->name = name;
		this->vertexA = vertexA;
		this->vertexB = vertexB;
		this->vertexC = vertexC;
		this->vertexD = vertexD;
		this->height = height;
	}
	
	Trapezoid::~Trapezoid(){
	
	}
	
	std::string Trapezoid::className() const{
		return "Trapezoid";
	}
	
	float Trapezoid::evaluate(float value) const{
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
	
	Trapezoid* Trapezoid::clone() const{
		return new Trapezoid(*this);
	}
	
	Trapezoid* Trapezoid::operator+(const Trapezoid &rhs) const{
		return new Trapezoid("", this->vertexA + rhs.vertexA, this->vertexB + rhs.vertexB, this->vertexC + rhs.vertexC, this->vertexD + rhs.vertexD);
		
	}
	
	Trapezoid* Trapezoid::operator-(const Trapezoid &rhs) const{
		return new Trapezoid("", this->vertexA - rhs.vertexA, this->vertexB - rhs.vertexB, this->vertexC - rhs.vertexC, this->vertexD - rhs.vertexD);
	}
	
	Trapezoid* Trapezoid::operator*(const float& rhs) const{
		return new Trapezoid("", this->vertexA * rhs, this->vertexB * rhs, this->vertexC * rhs, this->vertexD * rhs);
	}
	
	float Trapezoid::getVertexA() const{
		return vertexA;
	}
	
	void Trapezoid::setVertexA(float value){
		this->vertexA = value;
	}
	
	float Trapezoid::getVertexB() const{
		return vertexB;
	}
	
	void Trapezoid::setVertexB(float value){
		this->vertexB = value;
	}
	
	float Trapezoid::getVertexC() const{
		return vertexC;
	}
	
	void Trapezoid::setVertexC(float value){
		this->vertexC = value;
	}
	
	float Trapezoid::getVertexD() const{
		return vertexD;
	}
	
	void Trapezoid::setVertexD(float value){
		this->vertexD = value;
	}
	
}
