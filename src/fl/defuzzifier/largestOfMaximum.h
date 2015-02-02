#ifndef FUZZY_LARGEST_OF_MAXIMUM_H
#define FUZZY_LARGEST_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class largestOfMaximum : public defuzzifier{
    public:
        largestOfMaximum(float resolution = 0.1);
        virtual ~largestOfMaximum();
        
        virtual std::string className() const;
        virtual largestOfMaximum* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}
#endif