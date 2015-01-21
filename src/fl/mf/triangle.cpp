#include "triangle.h"

namespace fuzzy{
	triangle::triangle(std::string name, float vertexA, float vertexB, float vertexC, float height){
		this->name = name;
		this->vertexA = vertexA;
		this->vertexB = vertexB;
		this->vertexC = vertexC;
		this->height = height;
	}
	
	triangle::~triangle(){
	
	}
	
	std::string triangle::className() const{
		return "triangle";
	}
	
	float triangle::evaluate(float value) const{
		
	}
	
	triangle* triangle::clone() const{
	
	}
	
	float triangle::getVertexA() const{
		return vertexA;
	}
	
	void triangle::setVertexA(float value){
		this->vertexA = value;
	}
	
	float triangle::getVertexB() const{
		return vertexA;
	}
	
	void triangle::setVertexB(float value){
		this->vertexB = value;
	}
	
	float triangle::getVertexC() const{
		return vertexC;
	}
	
	void triangle::setVertexC(float value){
		this->vertexC = value;
	}
}