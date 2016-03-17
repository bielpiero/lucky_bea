#ifndef FUZZY_CENTROID_H
#define FUZZY_CENTROID_H

#include "defuzzifier.h"

namespace fuzzy{
    class centroid : public defuzzifier{
    public:
        centroid(float resolution = 0.1);
        virtual ~centroid();
        
        virtual std::string className() const;
        virtual centroid* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}

#endif