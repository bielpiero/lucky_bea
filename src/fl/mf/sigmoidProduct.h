#ifndef FUZZY_SIGMOID_PRODUCT_H
#define FUZZY_SIGMOID_PRODUCT_H

#include "mf.h"

namespace fuzzy{

	class sigmoidProduct : public mf{
	public:
		sigmoidProduct(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~sigmoidProduct();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sigmoidProduct* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif