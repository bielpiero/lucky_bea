#ifndef FUZZY_SNWEBER_H
#define FUZZY_SNWEBER_H

#include "snorm.h"

namespace fuzzy{
	class SNWeber : public SNorm{
		public:
			SNWeber(){}
			~SNWeber(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual SNWeber* clone() const;
	};
}


#endif