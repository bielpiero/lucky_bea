#include "centroid.h"
namespace fuzzy{
    centroid::centroid(float resolution):defuzzifier(resolution){
    
    }
    centroid::~centroid(){

    }

    std::string centroid::className() const{
        return "centroid";
    }
    
    centroid* centroid::clone() const{
        return new centroid(*this);
    }
    
    float centroid::defuzzify(const mf* membership, float minimum, float maximum) const{
        float result = fuzzy::nan;
        if(!stats::isInf(minimum) && !stats::isInf(maximum)){
            float iterations = std::abs((maximum - minimum) / this->resolution);
            float totalArea = 0;
            float totalX = 0;
            for (int i = 0; i < iterations; i++){
                float value = minimum + (i * resolution);
                float mfr = membership->evaluate(value);
                
                totalX += value * mfr;
                totalArea += mfr;
            }
            if(totalArea != 0){
                result = totalX / totalArea;
            } else {
                throw std::invalid_argument("Total area is zero in centroid defuzzification!");
            }
            
        }
        return result;
    }
}
