#ifndef FUZZY_SIGMOID_H
#define FUZZY_SIGMOID_H

#include "mf.h"

namespace fuzzy{

	class Sigmoid : public MF{
	public:
		Sigmoid(std::string name = "", 
				float inflection = fuzzy::inf, 
				float slope = fuzzy::inf, 
				float height = 1.0);
		virtual ~Sigmoid();
		
		virtual void setInflection(float value);
        virtual float getInflection() const;

        virtual void setSlope(float value);
        virtual float getSlope() const;
		
		virtual Sigmoid* operator+(const Sigmoid& rhs) const;
		virtual Sigmoid* operator-(const Sigmoid& rhs) const;
		
		virtual Sigmoid* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual Sigmoid* clone() const;
	private:
		float inflection;
		float slope;
	};
}
#endif