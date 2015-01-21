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
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = 0;
			if((vertexA == -fuzzy::nan) && (value < vertexB)){
				result = 1.0;
			} else if((value > vertexB) && (vertexC == fuzzy::nan)){
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
	
	triangle* triangle::clone() const{
		return new triangle(*this);
	}
	
	float triangle::getVertexA() const{
		return vertexA;
	}
	
	void triangle::setVertexA(float value){
		this->vertexA = value;
	}
	
	float triangle::getVertexB() const{
		return vertexB;
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