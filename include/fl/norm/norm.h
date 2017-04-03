#ifndef FUZZY_NORM_H
#define FUZZY_NORM_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

namespace fuzzy{
	class Norm{
		public:
			Norm(){}
			~Norm(){}
			virtual std::string className() const = 0;
			virtual float evaluate(float a, float b) const = 0;
			virtual Norm* clone() const = 0;
	};
}


#endif