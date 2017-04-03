#ifndef FUZZY_EXPRESSION_H
#define FUZZY_EXPRESSION_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

namespace fuzzy{
	class Expression{
		public:
			Expression(){}
		protected:
			virtual std::string className() const = 0;
			virtual float evaluate(float a, float b) const = 0;
			virtual Norm* clone() const = 0;
	};
}


#endif