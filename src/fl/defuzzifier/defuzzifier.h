#ifndef FUZZY_DEFUZZIFIER_H
#define FUZZY_DEFUZZIFIER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include "fl/constants.h"
#include "fl/mf/mf.h"

namespace fuzzy{
    class defuzzifier{
    protected:
        float resolution;
    public:
        defuzzifier(float resolution = 0.1){ this->resolution = resolution; }
        virtual ~defuzzifier(){}
        
        virtual float getResolution(){ return resolution; }
        virtual void setResolution(float resolution) { this->resolution = resolution; }
        
        virtual std::string className() const = 0;
        virtual defuzzifier* clone() const = 0;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const = 0;
    };
}

#endif