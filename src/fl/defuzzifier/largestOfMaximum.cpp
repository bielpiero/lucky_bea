#include "largestOfMaximum.h"

namespace fuzzy{
    largestOfMaximum::largestOfMaximum(float resolution):defuzzifier(resolution){
        
    }
    
    largestOfMaximum::~largestOfMaximum(){
        
    }

    std::string largestOfMaximum::className() const{
        return "largestOfMaximum";
    }
    
    largestOfMaximum* largestOfMaximum::clone() const{
        return new smallestOfMaximum(*this);
    }
    
    float largestOfMaximum::defuzzify(const mf* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        if(!stats::isInf(minimum) && !stats::isInf(maximum)){
            float iterations = std::abs((maximum - minimum) / this->resolution);
            std::vector<float> mfrs(iterations);
            std::vector<float> values(iterations);
            for (int i = 0; i < iterations; i++){
                float value = minimum + (i * resolution);
                float mfr = membership->evaluate(value);
                values.push_back(values);
                mfrs.push_back(mfr);
            }
            float maxMF = stats::max(mfrs);
            std::vector<int> indices = stats::findIndicesEqual(mfrs, maxMF);
            std::vector<float> maxXAbsValues;
            for (int i = 0; i < indices.size(); i++){
                maxXAbsValues.push_back(std::abs(values[indices[i]]));
            }
            
            result = stats::max(maxXAbsValues);
        }
        return result;
    }
}

