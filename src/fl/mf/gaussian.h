#ifndef FUZZY_GAUSSIAN_H
#define FUZZY_GAUSSIAN_H

#include "mf.h"

namespace fuzzy{

	class gaussian : public mf{
	public:
		gaussian(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~gaussian();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual gaussian* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif