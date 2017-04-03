#include "gaussProduct.h"

namespace fuzzy{
	GaussProduct::GaussProduct(std::string name, float meanA, float standardDeviationA, float meanB, float standardDeviationB, float height){
		this->name = name;
		this->height = height;
		
		this->meanA = meanA;
		this->standardDeviationA = standardDeviationA;
		
		this->meanB = meanB;
		this->standardDeviationB = standardDeviationB;
	}
	
	GaussProduct::~GaussProduct(){
	
	}
	
	std::string GaussProduct::className() const{
		return "GaussProduct";
	}
	
	float GaussProduct::evaluate(float value) const{
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
	
	GaussProduct* GaussProduct::operator+(const GaussProduct& rhs) const{
		return new GaussProduct("", this->meanA + rhs.meanA, this->standardDeviationA + rhs.standardDeviationA, this->meanB + rhs.meanB, this->standardDeviationB + rhs.standardDeviationB);
		
	}
	
	GaussProduct* GaussProduct::operator-(const GaussProduct& rhs) const{
		return new GaussProduct("", this->meanA - rhs.meanA, this->standardDeviationA - rhs.standardDeviationA, this->meanB - rhs.meanB, this->standardDeviationB - rhs.standardDeviationB);
	}
	
	GaussProduct* GaussProduct::operator*(const float& rhs) const{
		return new GaussProduct("", this->meanA * rhs, this->standardDeviationA * rhs, this->meanB * rhs, this->standardDeviationB * rhs);
	}
	
	GaussProduct* GaussProduct::clone() const{
		return new GaussProduct(*this);
	}
	
	float GaussProduct::getMeanA() const{
		return meanA;
	}
	
	void GaussProduct::setMeanA(float value){
		this->meanA = value;
	}
	
	float GaussProduct::getStandardDeviationA() const{
		return standardDeviationA;
	}
	
	void GaussProduct::setStandardDeviationA(float value){
		this->standardDeviationA = value;
	}
	
	float GaussProduct::getMeanB() const{
		return meanB;
	}
	
	void GaussProduct::setMeanB(float value){
		this->meanB = value;
	}
	
	float GaussProduct::getStandardDeviationB() const{
		return standardDeviationB;
	}
	
	void GaussProduct::setStandardDeviationB(float value){
		this->standardDeviationB = value;
	}
}