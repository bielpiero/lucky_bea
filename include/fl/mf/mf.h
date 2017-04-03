#ifndef FUZZY_MF_H
#define FUZZY_MF_H

#include <iostream>
#include <vector>
#include <string>

#include "stats.h"
#include "fl/constants.h"


namespace fuzzy{
    class MF{
    public:
        MF(const std::string name = "", float height = 1.0);		
        virtual ~MF();
        
        virtual std::string getName() const;
        virtual void setName(std::string name);
        
        virtual float getHeight() const;
        virtual void setHeight(float height);
		
	virtual MF* operator+(const MF &rhs) const;
	virtual MF* operator-(const MF &rhs) const;
	
	virtual MF* operator*(const float& rhs) const;

        virtual std::string className() const = 0;
        virtual float evaluate(float value) const = 0;
        virtual MF* clone() const = 0;
    protected:
        std::string name;
        float height;
    };
	

}

#endif
