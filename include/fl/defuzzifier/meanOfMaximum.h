#ifndef FUZZY_MEAN_OF_MAXIMUM_H
#define FUZZY_MEAN_OF_MAXIMUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class MeanOfMaximum : public Defuzzifier{
    public:
        MeanOfMaximum(float resolution = 0.1);
        virtual ~MeanOfMaximum();
        
        virtual std::string className() const;
        virtual MeanOfMaximum* clone() const { return new MeanOfMaximum(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}
#endif