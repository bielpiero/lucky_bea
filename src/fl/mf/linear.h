/* 
 * File:   linear.h
 * Author: bpalvarado
 *
 * Created on 26 de enero de 2015, 09:25 AM
 */

#ifndef FUZZY_LINEAR_H
#define	FUZZY_LINEAR_H

#include "mf.h"

namespace fuzzy{
    class linear : public mf{
    private:
        std::string name;
        std::vector<float> coeffs;
    public:
        linear(std::string name = "", std::vector<float> coeffs = std::vector<float>());
        virtual ~linear();
        
        virtual void setCoeffs(std::vector<float> coeffs);
        virtual std::vector<float> getCoeffs() const;
		
		virtual linear* operator+(const linear& rhs) const;
		virtual linear* operator-(const linear& rhs) const;
		
		virtual linear* operator*(const float& rhs) const;
        
        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual linear* clone() const;
        
    };
}



#endif	/* FUZZY_LINEAR_H */

