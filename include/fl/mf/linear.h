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
    class Linear : public MF{
    private:
        std::string name;
        std::vector<float> coeffs;
    public:
        Linear(std::string name = "", std::vector<float> coeffs = std::vector<float>());
        virtual ~Linear();
        
        virtual void setCoeffs(std::vector<float> coeffs);
        virtual std::vector<float> getCoeffs() const;
		
		virtual Linear* operator+(const Linear& rhs) const;
		virtual Linear* operator-(const Linear& rhs) const;
		
		virtual Linear* operator*(const float& rhs) const;
        
        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual Linear* clone() const;
        
    };
}



#endif	/* FUZZY_LINEAR_H */

