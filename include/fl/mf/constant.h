/* 
 * File:   constant.h
 * Author: bielpiero
 *
 * Created on February 2, 2015, 5:02 PM
 */

#ifndef FUZZY_CONSTANT_H
#define	FUZZY_CONSTANT_H

#include "mf.h"

namespace fuzzy {
    class Constant : public MF {
    private:
        float value;
    public:
        Constant(const std::string name = "", float value = fuzzy::nan);
        virtual ~Constant();
		
		virtual Constant* operator+(const Constant& rhs) const;
		virtual Constant* operator-(const Constant& rhs) const;
		
		virtual Constant* operator*(const float& rhs) const;
        
        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual Constant* clone() const;
    };
}
#endif	/* CONSTANT_H */

