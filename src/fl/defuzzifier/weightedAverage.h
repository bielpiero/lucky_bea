#ifndef FUZZY_WEIGHTED_AVG_H
#define FUZZY_WEIGHTED_AVG_H

#include "defuzzifier.h"

namespace fuzzy{
    class weightedAverage : defuzzifier {
	private:
		fuzzy::systemType type;
    public:
        weightedAverage(float resolution = 0.1, fuzzy::systemType type = TakagiSugeno);
        virtual ~weightedAverage();
        
        virtual std::string className() const;
        virtual weightedAverage* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}

#endif