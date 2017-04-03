#ifndef FUZZY_SNLUKASIEWICZ_H
#define FUZZY_SNLUKASIEWICZ_H

#include "snorm.h"

namespace fuzzy{
	class SNLukasiewicz : public SNorm{
		public:
			SNLukasiewicz(){}
			~SNLukasiewicz(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual SNLukasiewicz* clone() const;
	};
}


#endif