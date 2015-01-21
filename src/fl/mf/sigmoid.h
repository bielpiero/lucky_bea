#ifndef FUZZY_SIGMOID_H
#define FUZZY_SIGMOID_H

#include "mf.h"

namespace fuzzy{

	class sigmoid : public mf{
	public:
		sigmoid(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~sigmoid();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sigmoid* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif