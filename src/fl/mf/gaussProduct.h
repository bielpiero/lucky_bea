#ifndef FUZZY_GAUSS_PRODUCT_H
#define FUZZY_GAUSS_PRODUCT_H

#include "mf.h"

namespace fuzzy{

    class gaussProduct : public mf{
    public:
        gaussProduct(std::string name="", 
                        float meanA = fuzzy::inf, 
                        float standardDeviationA = fuzzy::inf, 
                        float meanB = fuzzy::inf, 
                        float standardDeviationB = fuzzy::inf, 
                        float height = 1.0);
        virtual ~gaussProduct();

        virtual float getMeanA() const;
        virtual void setMeanA(float value);

        virtual float getStandardDeviationA() const;
        virtual void setStandardDeviationA(float value);

        virtual float getMeanB() const;
        virtual void setMeanB(float value);

        virtual float getStandardDeviationB() const;
        virtual void setStandardDeviationB(float value);
		
		virtual gaussProduct* operator+(const gaussProduct& rhs) const;
		virtual gaussProduct* operator-(const gaussProduct& rhs) const;
		
		virtual gaussProduct* operator*(const float& rhs) const;

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual gaussProduct* clone() const;
    private:
        float meanA;
        float standardDeviationA;
        float meanB;
        float standardDeviationB;
    };
}
#endif