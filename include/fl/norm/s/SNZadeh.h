#ifndef FUZZY_SNZADEH_H
#define FUZZY_SNZADEH_H

#include "snorm.h"

namespace fuzzy{
	class SNZadeh : public SNorm{
		public:
			SNZadeh(){}
			~SNZadeh(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual SNZadeh* clone() const;
	};
}


#endif