#ifndef FUZZY_WEIGHTED_SUM_H
#define FUZZY_WEIGHTED_SUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class weightedSum : defuzzifier {
    public:
        weightedSum(fuzzy::systemType type = TakagiSugeno);
        virtual ~weightedSum();
        
        virtual std::string className() const;
        virtual weightedSum* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}

#endif