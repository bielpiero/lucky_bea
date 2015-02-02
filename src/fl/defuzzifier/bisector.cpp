#include "bisector.h"
namespace fuzzy{
    bisector::bisector(float resolution):defuzzifier(resolution){
    
    }
    bisector::~bisector(){

    }

    std::string bisector::className() const{
        return "bisector";
    }
    
    bisector* bisector::clone() const{
        return new centroid(*this);
    }
    
    float bisector::defuzzify(const mf* membership, float minimum, float maximum) const{
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
            float totalArea = stats::sum(mfrs);
            if(totalArea != 0){
                float current = 0;
                bool found = false;
                for (int i = 0; i < iterations && !found; i++){
                    current += mfrs[i];
                    if(current >= (totalArea / 2)){
                        found = true;
                        result = values[i];
                    }
                }
            } else {
                throw std::invalid_argument("Total area is zero in bisector defuzzification!");
            }
            
        }
        return result;
    }
}

