#include "linear.h"

namespace fuzzy{
    linear::linear(std::string name , std::vector<float> coeffs){
        this->coeffs = coeffs;
        this->name = name;
    }
    
    linear::~linear(){
        
    }
        
    std::string linear::className() const{
        
    }
    float linear::evaluate(float value) const{
        
    }
    linear* linear::clone() const{
        
    }
}
