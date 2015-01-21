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
	
	}
	
	float trapezoid::evaluate(float value) const{
		return "trapezoid";
	}
	
	trapezoid* trapezoid::clone() const{
	
	}
	
	float trapezoid::getVertexA() const{
		return vertexA;
	}
	
	void trapezoid::setVertexA(float value){
		this->vertexA = value;
	}
	
	float trapezoid::getVertexB() const{
		return vertexA;
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