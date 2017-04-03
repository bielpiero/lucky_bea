#ifndef FUZZY_WEIGHTED_AVG_H
#define FUZZY_WEIGHTED_AVG_H

#include "defuzzifier.h"

namespace fuzzy{
    class WeightedAverage : Defuzzifier {
	private:
		fuzzy::SystemType type;
    public:
        WeightedAverage(float resolution = 0.1, fuzzy::SystemType type = TakagiSugeno);
        virtual ~WeightedAverage();
        
        virtual std::string className() const;
        virtual WeightedAverage* clone() const { return new WeightedAverage(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}

#endif