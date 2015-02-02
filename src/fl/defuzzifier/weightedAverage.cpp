#include "weightedAverage.h"

namespace fuzzy{
    weightedAverage::weightedAverage(float resolution):defuzzifier(resolution){
    
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
