#ifndef FUZZY_SIGMOID_DIFFERENCE_H
#define FUZZY_SIGMOID_DIFFERENCE_H

#include "mf.h"

namespace fuzzy{

	class sigmoidDifference : public mf{
	public:
		sigmoidDifference(std::string name="", 
				float left = fuzzy::inf, 
				float rising = fuzzy::inf, 
				float falling = fuzzy::inf, 
				float right = fuzzy::inf,
				float height = 1.0);
		virtual ~sigmoidDifference();
		
		virtual void setLeft(float value);
        virtual float getLeft() const;

        virtual void setRising(float value);
        virtual float getRising() const;

        virtual void setFalling(float value);
        virtual float getFalling() const;

        virtual void setRight(float value);
        virtual float getRight() const;
		
		virtual sigmoidDifference* operator+(const sigmoidDifference* rhs) const;
		virtual sigmoidDifference* operator-(const sigmoidDifference* rhs) const;
		
		virtual sigmoidDifference* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sigmoidDifference* clone() const;
	private:
		float left;
		float rising;
		float falling;
		float right;
	};
}
#endif