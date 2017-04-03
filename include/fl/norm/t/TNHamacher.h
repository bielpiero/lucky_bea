#ifndef FUZZY_TNHAMACHER_H
#define FUZZY_TNHAMACHER_H

#include "tnorm.h"

namespace fuzzy{
	class TNHamacher : public TNorm{
		public:
			TNHamacher(){}
			~TNHamacher(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual TNHamacher* clone() const;
	};
}


#endif