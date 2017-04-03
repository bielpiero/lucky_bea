#include "meanOfMaximum.h"

namespace fuzzy{
    MeanOfMaximum::MeanOfMaximum(float resolution) : Defuzzifier(resolution){
        
    }
    
    MeanOfMaximum::~MeanOfMaximum(){
        
    }

    std::string MeanOfMaximum::className() const{
        return "MeanOfMaximum";
    }
    
    float MeanOfMaximum::defuzzify(const MF* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        if(!stats::isInf(minimum) && !stats::isInf(maximum)){
            float iterations = std::abs((maximum - minimum) / this->resolution);
            std::vector<float> mfrs(iterations);
            std::vector<float> values(iterations);
            for (int i = 0; i < iterations; i++){
                float value = minimum + (i * resolution);
                float mfr = membership->evaluate(value);
                values.push_back(value);
                mfrs.push_back(mfr);
            }
            float maxMF = stats::max(mfrs);
            std::vector<int> indices = stats::findIndicesEqual(mfrs, maxMF);
            std::vector<float> maxXValues;
            for (int i = 0; i < indices.size(); i++){
                maxXValues.push_back(values[indices[i]]);
            }
            
            result = stats::expectation(maxXValues);
        }
        return result;
    }
}

