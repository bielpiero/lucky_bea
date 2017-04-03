/* 
 * File:   constant.cpp
 * Author: bielpiero
 * 
 * Created on February 2, 2015, 5:02 PM
 */

#include "constant.h"
namespace fuzzy{
    Constant::Constant(const std::string name, float value) {
        this->name = name;
        this->value = value;
    }

    Constant::~Constant() {
        
    }
    
    std::string Constant::className() const{
        return "Constant";
    }
	
	Constant* Constant::operator+(const Constant& rhs) const{
		(void) rhs;
		
	}
	
	Constant* Constant::operator-(const Constant& rhs) const{
		(void) rhs;
	}
	
	Constant* Constant::operator*(const float& rhs) const{
		(void) rhs;
	}
    
    float Constant::evaluate(float value) const{
        (void) value;
        return this->value;
    }
    Constant* Constant::clone() const{
        return new Constant(*this);
    }
}

