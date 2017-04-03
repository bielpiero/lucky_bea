#ifndef FUZZY_TNLUKASIEWICZ_H
#define FUZZY_TNLUKASIEWICZ_H

#include "tnorm.h"

namespace fuzzy{
	class TNLukasiewicz : public TNorm{
		public:
			TNLukasiewicz() : TNorm(){}
			~TNLukasiewicz(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual TNLukasiewicz* clone() const;
	};
}


#endif