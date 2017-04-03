#include "weightedSum.h"

namespace fuzzy{
    WeightedSum::WeightedSum(float resolution, fuzzy::SystemType type) : Defuzzifier(resolution){
		this->type = type;
    }

    WeightedSum::~WeightedSum(){

    }

    std::string WeightedSum::className() const{
        return "WeightedSum";
    }
    
    float WeightedSum::defuzzify(const MF* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        
        return result;
    }
}

