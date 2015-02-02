#include "linear.h"

namespace fuzzy{
    linear::linear(std::string name , std::vector<float> coeffs){
        this->coeffs = coeffs;
        this->name = name;
    }
    
    linear::~linear(){
        
    }
    
    void linear::setCoeffs(std::vector<float> coeffs){
        this->coeffs = coeffs;
    }
    
    std::vector<float> linear::getCoeffs() const{
        return coeffs;
    }
        
    std::string linear::className() const{
        return "linear";
    }
    
    float linear::evaluate(float value) const{
        (void) value;
    }
    
    linear* linear::clone() const{
        return new linear(*this);
    }
}
