#include "linear.h"

namespace fuzzy{
    Linear::Linear(std::string name , std::vector<float> coeffs){
        this->coeffs = coeffs;
        this->name = name;
    }
    
    Linear::~Linear(){
        
    }
    
    void Linear::setCoeffs(std::vector<float> coeffs){
        this->coeffs = coeffs;
    }
    
    std::vector<float> Linear::getCoeffs() const{
        return coeffs;
    }
        
    std::string Linear::className() const{
        return "Linear";
    }
    
    float Linear::evaluate(float value) const{
        (void) value;
    }
	
	Linear* Linear::operator+(const Linear& rhs) const{
		(void) rhs;
	
	}

	Linear* Linear::operator-(const Linear& rhs) const{
		(void) rhs;
	}

	Linear* Linear::operator*(const float& rhs) const{
		(void) rhs;
	}
    
    Linear* Linear::clone() const{
        return new Linear(*this);
    }
}
