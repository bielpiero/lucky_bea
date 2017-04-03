#ifndef FUZZY_GAUSS_PRODUCT_H
#define FUZZY_GAUSS_PRODUCT_H

#include "mf.h"

namespace fuzzy{

    class GaussProduct : public MF{
    public:
        GaussProduct(std::string name="", 
                        float meanA = fuzzy::inf, 
                        float standardDeviationA = fuzzy::inf, 
                        float meanB = fuzzy::inf, 
                        float standardDeviationB = fuzzy::inf, 
                        float height = 1.0);
        virtual ~GaussProduct();

        virtual float getMeanA() const;
        virtual void setMeanA(float value);

        virtual float getStandardDeviationA() const;
        virtual void setStandardDeviationA(float value);

        virtual float getMeanB() const;
        virtual void setMeanB(float value);

        virtual float getStandardDeviationB() const;
        virtual void setStandardDeviationB(float value);
		
		virtual GaussProduct* operator+(const GaussProduct& rhs) const;
		virtual GaussProduct* operator-(const GaussProduct& rhs) const;
		
		virtual GaussProduct* operator*(const float& rhs) const;

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual GaussProduct* clone() const;
    private:
        float meanA;
        float standardDeviationA;
        float meanB;
        float standardDeviationB;
    };
}
#endif