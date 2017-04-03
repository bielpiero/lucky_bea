#ifndef FUZZY_TNZADEH_H
#define FUZZY_TNZADEH_H

#include "tnorm.h"

namespace fuzzy{
	class TNZadeh : public TNorm{
		public:
			TNZadeh(){}
			~TNZadeh(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual TNZadeh* clone() const;
	};
}
#endif