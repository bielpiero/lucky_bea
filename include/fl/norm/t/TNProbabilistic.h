#ifndef FUZZY_TNPROBABILISTIC_H
#define FUZZY_TNPROBABILISTIC_H

#include "tnorm.h"

namespace fuzzy{
	class TNProbabilistic : public TNorm{
		public:
			TNProbabilistic() : TNorm(){}
			~TNProbabilistic(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual TNProbabilistic* clone() const;
	};
}


#endif