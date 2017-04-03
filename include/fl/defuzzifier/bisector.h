#ifndef FUZZY_BISERCTOR_H
#define FUZZY_BISERCTOR_H

#include "defuzzifier.h"

namespace fuzzy{
    class Bisector : public Defuzzifier{
        
    public:
        Bisector(float resolution = 0.1);
        virtual ~Bisector();
        
        virtual std::string className() const;
        virtual Bisector* clone() const { return new Bisector(*this); }
        virtual float defuzzify(const MF* membership, float minimum, float maximum) const;
    };
}

#endif