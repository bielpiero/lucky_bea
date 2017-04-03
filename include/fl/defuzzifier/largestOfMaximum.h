#ifndef FUZZY_LARGEST_OF_MAXIMUM_H
#define FUZZY_LARGEST_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class LargestOfMaximum : public Defuzzifier{
    public:
        LargestOfMaximum(float resolution = 0.1);
        virtual ~LargestOfMaximum();
        
        virtual std::string className() const;
        virtual LargestOfMaximum* clone() const { return new LargestOfMaximum(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}
#endif