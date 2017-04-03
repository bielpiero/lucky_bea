#ifndef FUZZY_GAUSS_BELL_H
#define FUZZY_GAUSS_BELL_H

#include "mf.h"

namespace fuzzy{

    class GaussBell : public MF{
    public:
        GaussBell(std::string name="", 
                        float center = fuzzy::inf, 
                        float width = fuzzy::inf, 
                        float slope = fuzzy::inf, 
                        float height = 1.0);
        virtual ~GaussBell();
		


        virtual float getCenter() const;
        virtual void setCenter(float value);

        virtual float getWidth() const;
        virtual void setWidth(float value);

        virtual float getSlope() const;
        virtual void setSlope(float value);
		
		virtual GaussBell* operator+(const GaussBell& rhs) const;
		virtual GaussBell* operator-(const GaussBell& rhs) const;
		
		virtual GaussBell* operator*(const float& rhs) const;

        virtual std::string className() const;
        virtual float evaluate(float value) const;
        virtual GaussBell* clone() const;
    private:
        float center;
        float width;
        float slope;
    };
}
#endif