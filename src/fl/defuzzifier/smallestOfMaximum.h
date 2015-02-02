#ifndef FUZZY_SMALLEST_OF_MAXIMUM_H
#define FUZZY_SMALLEST_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class smallestOfMaximum : public defuzzifier{
    public:
        smallestOfMaximum(float resolution = 0.1);
        virtual ~smallestOfMaximum();
        
        virtual std::string className() const;
        virtual smallestOfMaximum* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}
#endif