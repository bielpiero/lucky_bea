#ifndef FUZZY_GAUSS_BELL_H
#define FUZZY_GAUSS_BELL_H

#include "mf.h"

namespace fuzzy{

    class gaussBell : public mf{
    public:
        gaussBell(std::string name="", 
                        float center = fuzzy::inf, 
                        float width = fuzzy::inf, 
                        float slope = fuzzy::inf, 
                        float height = 1.0);
        virtual ~gaussBell();
		


        virtual float getCenter() const;
        virtual void setCenter(float value);

        virtual float getWidth() const;
        virtual void setWidth(float value);

        virtual float getSlope() const;
        virtual void setSlope(float value);
		
		virtual gaussBell* operator+(const gaussBell* rhs) const;
		virtual gaussBell* operator-(const gaussBell* rhs) const;
		
		virtual gaussBell* operator*(const float& rhs) const;

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual gaussBell* clone() const;
    private:
        float center;
        float width;
        float slope;
    };
}
#endif