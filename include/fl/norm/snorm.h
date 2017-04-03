#ifndef FUZZY_SNORM_H
#define FUZZY_SNORM_H

#include "norm.h"

namespace fuzzy{
	class SNorm : public Norm{
		public:
			SNorm(){}
			~SNorm(){}
			virtual SNorm* clone() const = 0;
	};
}


#endif