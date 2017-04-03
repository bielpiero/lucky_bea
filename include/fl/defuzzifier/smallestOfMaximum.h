#ifndef FUZZY_SMALLEST_OF_MAXIMUM_H
#define FUZZY_SMALLEST_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class SmallestOfMaximum : public Defuzzifier{
    public:
        SmallestOfMaximum(float resolution = 0.1);
        virtual ~SmallestOfMaximum();
        
        virtual std::string className() const;
        virtual SmallestOfMaximum* clone() const { return new SmallestOfMaximum(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}
#endif