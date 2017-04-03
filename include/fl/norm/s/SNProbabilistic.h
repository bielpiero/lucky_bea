#ifndef FUZZY_SNPROBABILISTIC_H
#define FUZZY_SNPROBABILISTIC_H

#include "snorm.h"

namespace fuzzy{
	class SNProbabilistic : public SNorm{
		public:
			SNProbabilistic(){}
			~SNProbabilistic(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual SNProbabilistic* clone() const;
	};
}


#endif