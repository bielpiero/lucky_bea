#ifndef FUZZY_WEIGHTED_SUM_H
#define FUZZY_WEIGHTED_SUM_H

#include "defuzzifier.h"

namespace fuzzy{
    class WeightedSum : Defuzzifier {
	private:
		fuzzy::SystemType type;
    public:
        WeightedSum(float resolution = 0.1, fuzzy::SystemType type = TakagiSugeno);
        virtual ~WeightedSum();
        
        virtual std::string className() const;
        virtual WeightedSum* clone() const { return new WeightedSum(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}

#endif