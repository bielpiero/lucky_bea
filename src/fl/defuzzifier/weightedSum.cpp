#include "weightedSum.h"

namespace fuzzy{
    weightedSum::weightedSum(float resolution, fuzzy::systemType type):defuzzifier(resolution){
		this->type = type;
    }
    weightedSum::~weightedSum(){

    }

    std::string weightedSum::className() const{
        return "weightedSum";
    }
    
    weightedSum* weightedSum::clone() const{
        return new weightedSum(*this);
    }
    
    float weightedSum::defuzzify(const mf* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        
        return result;
    }
}

