#ifndef FUZZY_GAUSSIAN_H
#define FUZZY_GAUSSIAN_H

#include "mf.h"

namespace fuzzy{

    class Gaussian : public MF{
    public:
        Gaussian(std::string name="", 
                        float mean = fuzzy::inf, 
                        float standardDeviation = fuzzy::inf, 
                        float height = 1.0);
        virtual ~Gaussian();
        
        virtual float getMean() const;
        virtual void setMean(float value);

        virtual float getStandardDeviation() const;
        virtual void setStandardDeviation(float value);
		
	virtual Gaussian* operator+(const Gaussian& rhs) const;
	virtual Gaussian* operator-(const Gaussian& rhs) const;
		
	virtual Gaussian* operator*(const float& rhs) const;

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual Gaussian* clone() const;
    private:
        float mean;
        float standardDeviation;
    };
}
#endif
