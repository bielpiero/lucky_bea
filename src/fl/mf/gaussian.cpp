#include "gaussian.h"

namespace fuzzy{
	gaussian::gaussian(std::string name, float mean, float standardDeviation, float height){
		this->name = name;
		this->height = height;
		
		this->mean = mean;
		this->standardDeviation = standardDeviation;
		
	}
	
	gaussian::~gaussian(){
	
	}
	
	std::string gaussian::className() const{
		return "gaussian";
	}
	
	float gaussian::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = std::exp((-std::pow(value - mean, 2))/(std::pow(2 * standardDeviation, 2)));
		} 
		
		return (result * height);
	}
	
	gaussian* gaussian::clone() const{
		return new gaussian(*this);
	}
	
	float gaussian::getMean() const{
		return mean;
	}
	
	void gaussian::setMean(float value){
		this->mean = value;
	}
	
	float gaussian::getStandardDeviation() const{
		return standardDeviation;
	}
	
	void gaussian::setStandardDeviation(float value){
		this->standardDeviation = value;
	}
}