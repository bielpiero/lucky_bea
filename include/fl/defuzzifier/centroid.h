#ifndef FUZZY_CENTROID_H
#define FUZZY_CENTROID_H

#include "defuzzifier.h"

namespace fuzzy{
    class Centroid : public Defuzzifier{
    public:
        Centroid(float resolution = 0.1);
        virtual ~Centroid();
        
        virtual std::string className() const;
        virtual Centroid* clone() const { return new Centroid(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}

#endif