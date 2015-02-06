#include "weightedAverage.h"

namespace fuzzy{
    weightedAverage::weightedAverage(float resolution, fuzzy::systemType type):defuzzifier(resolution){
		this->type = type;
    }
    weightedAverage::~weightedAverage(){

    }

    std::string weightedAverage::className() const{
        return "weightedAverage";
    }
    
    weightedAverage* weightedAverage::clone() const{
        return new weightedAverage(*this);
    }
    
    float weightedAverage::defuzzify(const mf* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        
        return result;
    }
}
