#include "gaussProduct.h"

namespace fuzzy{
	gaussProduct::gaussProduct(std::string name, float meanA, float standardDeviationA, float meanB, float standardDeviationB, float height){
		this->name = name;
		this->height = height;
		
		this->meanA = meanA;
		this->standardDeviationA = standardDeviationA;
		
		this->meanB = meanB;
		this->standardDeviationB = standardDeviationB;
	}
	
	gaussProduct::~gaussProduct(){
	
	}
	
	std::string gaussProduct::className() const{
		return "gaussProduct";
	}
	
	float gaussProduct::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			bool meanAIndex = (value <= meanA);
			bool meanBIndex = (value >= meanB);
			float f1 = (1 - meanAIndex) + meanAIndex * std::exp((-std::pow(value - meanA, 2))/(std::pow(2 * standardDeviationA, 2)));
			float f2 = (1 - meanBIndex) + meanBIndex * std::exp((-std::pow(value - meanB, 2))/(std::pow(2 * standardDeviationB, 2)));
			result = f1 * f2;
		} 
		return (result * height);
	}
	
	gaussProduct* gaussProduct::operator+(const gaussProduct& rhs) const{
		return new gaussProduct("", this->meanA + rhs.meanA, this->standardDeviationA + rhs.standardDeviationA, this->meanB + rhs.meanB, this->standardDeviationB + rhs.standardDeviationB);
		
	}
	
	gaussProduct* gaussProduct::operator-(const gaussProduct& rhs) const{
		return new gaussProduct("", this->meanA - rhs.meanA, this->standardDeviationA - rhs.standardDeviationA, this->meanB - rhs.meanB, this->standardDeviationB - rhs.standardDeviationB);
	}
	
	gaussProduct* gaussProduct::operator*(const float& rhs) const{
		return new gaussProduct("", this->meanA * rhs, this->standardDeviationA * rhs, this->meanB * rhs, this->standardDeviationB * rhs);
	}
	
	gaussProduct* gaussProduct::clone() const{
		return new gaussProduct(*this);
	}
	
	float gaussProduct::getMeanA() const{
		return meanA;
	}
	
	void gaussProduct::setMeanA(float value){
		this->meanA = value;
	}
	
	float gaussProduct::getStandardDeviationA() const{
		return standardDeviationA;
	}
	
	void gaussProduct::setStandardDeviationA(float value){
		this->standardDeviationA = value;
	}
	
	float gaussProduct::getMeanB() const{
		return meanB;
	}
	
	void gaussProduct::setMeanB(float value){
		this->meanB = value;
	}
	
	float gaussProduct::getStandardDeviationB() const{
		return standardDeviationB;
	}
	
	void gaussProduct::setStandardDeviationB(float value){
		this->standardDeviationB = value;
	}
}