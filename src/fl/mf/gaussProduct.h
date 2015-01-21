#ifndef FUZZY_GAUSS_PRODUCT_H
#define FUZZY_GAUSS_PRODUCT_H

#include "mf.h"

namespace fuzzy{

	class gaussProduct : public mf{
	public:
		gaussProduct(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~gaussProduct();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual gaussProduct* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif