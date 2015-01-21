#ifndef FUZZY_SIGMOID_DIFFERENCE_H
#define FUZZY_SIGMOID_DIFFERENCE_H

#include "mf.h"

namespace fuzzy{

	class sigmoidDifference : public mf{
	public:
		sigmoidDifference(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~sigmoidDifference();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sigmoidDifference* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif