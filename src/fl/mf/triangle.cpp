#include "triangle.h"

namespace fuzzy{
	Triangle::Triangle(std::string name, float vertexA, float vertexB, float vertexC, float height){
		this->name = name;
		this->height = height;
		
		this->vertexA = vertexA;
		this->vertexB = vertexB;
		this->vertexC = vertexC;
	}
	
	Triangle::~Triangle(){
	
	}
	
	std::string Triangle::className() const{
		return "Triangle";
	}
	
	float Triangle::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 0;
			if((vertexA == -fuzzy::inf) && (value <= vertexB)){
				result = 1.0;
			} else if((value > vertexB) && (vertexC == fuzzy::inf)){
				result = 1.0;
			} else {
				std::vector<float> minv(2);
				minv[0] = (value-vertexA)/(vertexB-vertexA);
				minv[1] = (vertexC-value)/(vertexC-vertexB);
				
				std::vector<float> maxv(2);
				maxv[0] = stats::min(minv);
				maxv[1] = 0;
				
				result = stats::max(maxv);
			}
		} 
		
		return (result * height);
	}
	
	Triangle* Triangle::clone() const{
		return new Triangle(*this);
	}
	
	Triangle* Triangle::operator+(const Triangle& rhs) const{
		return new Triangle("", this->vertexA + rhs.vertexA, this->vertexB + rhs.vertexB, this->vertexC + rhs.vertexC);
		
	}
	
	Triangle* Triangle::operator-(const Triangle& rhs) const{
		return new Triangle("", this->vertexA - rhs.vertexA, this->vertexB - rhs.vertexB, this->vertexC - rhs.vertexC);
	}
	
	Triangle* Triangle::operator*(const float& rhs) const{
		return new Triangle("", this->vertexA * rhs, this->vertexB * rhs, this->vertexC * rhs);
	}
	
	float Triangle::getVertexA() const{
		return vertexA;
	}
	
	void Triangle::setVertexA(float value){
		this->vertexA = value;
	}
	
	float Triangle::getVertexB() const{
		return vertexB;
	}
	
	void Triangle::setVertexB(float value){
		this->vertexB = value;
	}
	
	float Triangle::getVertexC() const{
		return vertexC;
	}
	
	void Triangle::setVertexC(float value){
		this->vertexC = value;
	}
}