#ifndef FUZZY_TNORM_H
#define FUZZY_TNORM_H

#include "norm.h"

namespace fuzzy{
	class TNorm : public Norm{
		public:
			TNorm(){}
			~TNorm(){}
			virtual TNorm* clone() const = 0;
	};
}


#endif