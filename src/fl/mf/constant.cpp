/* 
 * File:   constant.cpp
 * Author: bielpiero
 * 
 * Created on February 2, 2015, 5:02 PM
 */

#include "constant.h"
namespace fuzzy{
    constant::constant(const std::string name, float value) {
        this->name = name;
        this->value = value;
    }

    constant::~constant() {
        
    }
    
    std::string constant::className() const{
        return "constant";
    }
    
    float constant::evaluate(float value) const{
        (void) value;
        return this->value;
    }
    constant* constant::clone() const{
        return new constant(*this);
    }
}
