#ifndef FUZZY_BISERCTOR_H
#define FUZZY_BISERCTOR_H

#include "defuzzifier.h"

namespace fuzzy{
    class bisector : public defuzzifier{
        
    public:
        bisector(float resolution = 0.1);
        virtual ~bisector();
        
        virtual std::string className() const;
        virtual bisector* clone() const;
        virtual float defuzzify(const mf* membership, float minimum, float maximum) const;
    };
}

#endif