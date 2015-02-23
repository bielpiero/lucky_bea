#ifndef FUZZY_SIGMOID_H
#define FUZZY_SIGMOID_H

#include "mf.h"

namespace fuzzy{

	class sigmoid : public mf{
	public:
		sigmoid(std::string name="", 
				float inflection = fuzzy::inf, 
				float slope = fuzzy::inf, 
				float height = 1.0);
		virtual ~sigmoid();
		
		virtual void setInflection(float value);
        virtual float getInflection() const;

        virtual void setSlope(float value);
        virtual float getSlope() const;
		
		virtual sigmoid* operator+(const sigmoid* rhs) const;
		virtual sigmoid* operator-(const sigmoid* rhs) const;
		
		virtual sigmoid* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sigmoid* clone() const;
	private:
		float inflection;
		float slope;
	};
}
#endif