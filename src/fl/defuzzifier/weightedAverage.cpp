#include "weightedAverage.h"

namespace fuzzy{
    WeightedAverage::WeightedAverage(float resolution, fuzzy::SystemType type) : Defuzzifier(resolution){
		this->type = type;
    }
    WeightedAverage::~WeightedAverage(){

    }

    std::string WeightedAverage::className() const{
        return "WeightedAverage";
    }
    
    float WeightedAverage::defuzzify(const MF* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        
        return result;
    }
}
