#include "gaussian.h"

namespace fuzzy{
	Gaussian::Gaussian(std::string name, float mean, float standardDeviation, float height){
		this->name = name;
		this->height = height;
		
		this->mean = mean;
		this->standardDeviation = standardDeviation;
		
	}
	
	Gaussian::~Gaussian(){
	
	}
	
	std::string Gaussian::className() const{
		return "Gaussian";
	}
	
	float Gaussian::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			result = std::exp((-std::pow(value - mean, 2))/(std::pow(2 * standardDeviation, 2)));
		} 
		
		return (result * height);
	}
	
	Gaussian* Gaussian::clone() const{
		return new Gaussian(*this);
	}
	
	Gaussian* Gaussian::operator+(const Gaussian& rhs) const{
		return new Gaussian("", this->mean + rhs.mean, this->standardDeviation + rhs.standardDeviation);
		
	}
	
	Gaussian* Gaussian::operator-(const Gaussian& rhs) const{
		return new Gaussian("", this->mean - rhs.mean, this->standardDeviation - rhs.standardDeviation);
	}
	
	Gaussian* Gaussian::operator*(const float& rhs) const{
		return new Gaussian("", this->mean * rhs, this->standardDeviation * rhs);
	}
	
	float Gaussian::getMean() const{
		return mean;
	}
	
	void Gaussian::setMean(float value){
		this->mean = value;
	}
	
	float Gaussian::getStandardDeviation() const{
		return standardDeviation;
	}
	
	void Gaussian::setStandardDeviation(float value){
		this->standardDeviation = value;
	}
}