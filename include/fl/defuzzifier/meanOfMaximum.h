#ifndef FUZZY_MEAN_OF_MAXIMUM_H
#define FUZZY_MEAN_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class meanOfMaximum : public defuzzifier{
    public:
        meanOfMaximum(float resolution = 0.1);
        virtual ~meanOfMaximum();
        
        virtual std::string className() const;
        virtual meanOfMaximum* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}
#endif