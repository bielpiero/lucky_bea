#ifndef FUZZY_MF_H
#define FUZZY_MF_H

#include <iostream>
#include <vector>
#include <string>

#include "stats.h"
#include "fl/constants.h"


namespace fuzzy{
    class mf{
    public:
        mf(const std::string name = "", float height = 1.0);		
        virtual ~mf();
        
        virtual std::string getName() const;
        virtual void setName(std::string name);
        
        virtual float getHeight() const;
        virtual void setHeight(float height);
		
		virtual mf* operator+(const mf &rhs) const = 0;
		virtual mf* operator-(const mf &rhs) const = 0;
		
		virtual mf* operator*(const float& rhs) const = 0;

        virtual std::string className() const = 0;
        virtual float evaluate(float value) const = 0;
        virtual mf* clone() const = 0;
    protected:
        std::string name;
        float height;
    };
	

}

#endif