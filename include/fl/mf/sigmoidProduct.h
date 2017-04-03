#ifndef FUZZY_SIGMOID_PRODUCT_H
#define FUZZY_SIGMOID_PRODUCT_H

#include "mf.h"

namespace fuzzy{

	class SigmoidProduct : public MF{
	public:
		SigmoidProduct(std::string name = "", 
				float left = fuzzy::inf, 
				float rising = fuzzy::inf, 
				float falling = fuzzy::inf, 
				float right = fuzzy::inf,
				float height = 1.0);
		virtual ~SigmoidProduct();
		
		virtual void setLeft(float value);
        virtual float getLeft() const;

        virtual void setRising(float value);
        virtual float getRising() const;

        virtual void setFalling(float value);
        virtual float getFalling() const;

        virtual void setRight(float value);
        virtual float getRight() const;
		
		virtual SigmoidProduct* operator+(const SigmoidProduct& rhs) const;
		virtual SigmoidProduct* operator-(const SigmoidProduct& rhs) const;
		
		virtual SigmoidProduct* operator*(const float& rhs) const;
				
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual SigmoidProduct* clone() const;
	private:
		float left;
		float rising;
		float falling;
		float right;
	};
}
#endif