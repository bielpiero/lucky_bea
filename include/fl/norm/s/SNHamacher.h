#ifndef FUZZY_SNHAMACHER_H
#define FUZZY_SNHAMACHER_H

#include "snorm.h"

namespace fuzzy{
	class SNHamacher : public SNorm{
		public:
			SNHamacher(){}
			~SNHamacher(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual SNHamacher* clone() const;
	};
}


#endif