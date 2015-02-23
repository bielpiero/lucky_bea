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
    class constant : public mf {
    private:
        float value;
    public:
        constant(const std::string name = "", float value = fuzzy::nan);
        virtual ~constant();
		
		virtual constant* operator+(const constant* rhs) const;
		virtual constant* operator-(const constant* rhs) const;
		
		virtual constant* operator*(const float& rhs) const;
        
        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual constant* clone() const;
    };
}
#endif	/* CONSTANT_H */

