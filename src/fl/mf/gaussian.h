#ifndef FUZZY_GAUSSIAN_H
#define FUZZY_GAUSSIAN_H

#include "mf.h"

namespace fuzzy{

    class gaussian : public mf{
    public:
        gaussian(std::string name="", 
                        float mean = fuzzy::inf, 
                        float standardDeviation = fuzzy::inf, 
                        float height = 1.0);
        virtual ~gaussian();
        
        virtual float getMean() const;
        virtual void setMean(float value);

        virtual float getStandardDeviation() const;
        virtual void setStandardDeviation(float value);

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual gaussian* clone() const;
    private:
        float mean;
        float standardDeviation;
    };
}
#endif